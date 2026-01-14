

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <nuttx/mutex.h>
#include "mindgrove_spi.h"
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <debug.h>
#include <arch/board/board.h>
#include "riscv_internal.h"

#define MG_SPI0_BASE 0x20000u
#define MG_SPI1_BASE 0x20100u
#define MG_SPI2_BASE 0x20200u
#define MG_SPI3_BASE 0x20300u

// struct mg_spi_priv_s
// {
//   struct spi_dev_s spi;     /* MUST be first */

//   uintptr_t        hw_base;    /* SPI base address */

//   bool             enabled;

//   uint32_t         frequency;
//   uint32_t         actual;

//   enum spi_mode_e  mode;
//   uint8_t          nbits;

//   uint32_t         devid;   /* CS device ID */

//   mutex_t          lock;    /* SPI bus lock */
// };


#ifdef CONFIG_SPI_HWFEATURES
typedef uint8_t spi_hwfeatures_t;
#endif

struct mg_spi_priv_s
{
  /* Interfaces */
  struct spi_dev_s spi; /* Master interface (MUST be first) */

#ifdef CONFIG_SPI_SLAVE
  struct spi_slave_ctrlr_s slave_ctrlr; /* Slave interface */
  struct spi_slave_dev_s *sdev;         /* Upper-half callback pointer */
#endif

  /* Hardware State */
  uintptr_t hw_base; /* SPI base address */
  bool enabled;
  bool is_slave; /* Flag to track current hardware mode */

  /* Configuration */
  uint32_t frequency;
  uint32_t actual;
  enum spi_mode_e mode;
  uint8_t nbits;

  uint32_t devid; /* CS device ID */
  mutex_t lock;   /* SPI bus lock */
};

// #ifdef CONFIG_SPI_SLAVE
// static void mg_spi_slave_bind(FAR struct spi_slave_ctrlr_s *ctrlr,
//                               FAR struct spi_slave_dev_s *sdev,
//                               enum spi_slave_mode_e mode, int nbits);
// static void mg_spi_slave_unbind(FAR struct spi_slave_ctrlr_s *ctrlr);
// static int mg_spi_slave_enqueue(FAR struct spi_slave_ctrlr_s *ctrlr,
//                                 FAR const void *data, size_t nwords);
// static bool mg_spi_slave_qfull(FAR struct spi_slave_ctrlr_s *ctrlr);
// static void mg_spi_slave_qflush(FAR struct spi_slave_ctrlr_s *ctrlr);
// static size_t mg_spi_slave_qpoll(FAR struct spi_slave_ctrlr_s *ctrlr);
// #endif

static int mg_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t mg_spi_setfrequency(FAR struct spi_dev_s *dev,
                                    uint32_t frequency);
static void mg_spi_setmode(FAR struct spi_dev_s *dev,
                           enum spi_mode_e mode);
static void mg_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint8_t mg_spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
static void mg_spi_exchange(FAR struct spi_dev_s *dev,
                            FAR const void *txbuffer,
                            FAR void *rxbuffer,
                            size_t nwords);
static void mg_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected);
static int mg_spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features);
struct mg_spi_config_s
{
  uint32_t clk_freq;    /* SPI clock frequency */
  enum spi_mode_e mode; /* SPI default mode */
  bool use_irq;         /* Use DMA */
};

static const struct spi_ops_s g_mg_spi_ops =
    {
        .lock = mg_spi_lock,
        .select = mg_spi_select, /* ADD THIS LINE */
        .setfrequency = mg_spi_setfrequency,
        .setmode = mg_spi_setmode,
        .setbits = mg_spi_setbits,
        .status = mg_spi_status,
        .exchange = mg_spi_exchange,
        #ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = mg_spi_hwfeatures, /* THIS LINKS YOUR FUNCTION */
#endif
};
#ifdef CONFIG_SPI_SLAVE
static const struct spi_slave_ctrlrops_s g_mg_spi_slave_ops =
    {
        .bind = mg_spi_slave_bind,
        .unbind = mg_spi_slave_unbind,
        .enqueue = mg_spi_slave_enqueue,
        .qfull = mg_spi_slave_qfull,
        .qflush = mg_spi_slave_qflush,
        .qpoll = mg_spi_slave_qpoll, /* Not typically used if you have interrupts */
};
#endif

static struct mg_spi_priv_s g_mg_spi0_priv =
    {
        .spi =
            {
                .ops = &g_mg_spi_ops,
            },
        .hw_base = MG_SPI0_BASE,
        .enabled = false,
        .lock = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi1_priv =
    {
        .spi =
            {
                .ops = &g_mg_spi_ops,
            },
        .hw_base = MG_SPI1_BASE,
        .enabled = false,
        .lock = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi2_priv =
    {
        .spi =
            {
                .ops = &g_mg_spi_ops,
            },
        .hw_base = MG_SPI2_BASE,
        .enabled = false,
        .lock = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi3_priv =
    {
        .spi =
            {
                .ops = &g_mg_spi_ops,
            },
        .hw_base = MG_SPI3_BASE,
        .enabled = false,
        .lock = NXMUTEX_INITIALIZER,
};

// static void mg_spi_slave_bind(FAR struct spi_slave_ctrlr_s *ctrlr,
//                               FAR struct spi_slave_dev_s *sdev,
//                               enum spi_slave_mode_e mode, int nbits)
// {
//   struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)ctrlr;

//   /* 1. Save the callback pointer (sdev) so we can use it in our ISR */
//   priv->sdev = sdev;
//   priv->is_slave = true;

//   /* 2. Hardware Switch: Set the SPI peripheral to Slave Mode */
//   // Example: Setting a bit in your CONTROL register
//   modifyreg32(MG_SPI_CONTROL, MG_SPI_MODE, MG_SPI_SLAVE_MODE);

//   /* 3. Configure the hardware using your existing master-logic functions */
//   modifyreg32(MG_SPI_CONTROL,
//               0,
//               MG_SPI_MISO_OUTEN_MASK);

//   modifyreg32(MG_SPI_CONTROL, MG_SPI_SCLK_OUTEN_MASK | MG_SPI_NCS_OUTEN_MASK | MG_SPI_MOSI_OUTEN_MASK, 0);

//   mg_spi_setmode((struct spi_dev_s *)priv, (enum spi_mode_e)mode);
//   mg_spi_setbits((struct spi_dev_s *)priv, nbits);

//   /* 4. Let the upper half know the Slave is now selected/ready */
//   SPIS_DEV_SELECT(sdev, true);
// }


// static int mg_spi_slave_enqueue(FAR struct spi_slave_ctrlr_s *ctrlr,
//                                 FAR const void *data, size_t nwords)
// {

//   struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)ctrlr;

//   // printf(" nwords %d",nwords);
//   uint8_t comm_mode = 0; // tx
//   modifyreg32(MG_SPI_CONTROL, MG_SPI_COMM_MODE_MASK, (uint32_t)comm_mode << MG_SPI_COMM_MODE_SHIFT);

//   size_t i;

//   switch (priv->nbits)
//   {

//   case 8:
//     const uint8_t *tx_8 = (const uint8_t *)data;
//     for (i = 0; i < nwords; i++)
//     {
//       while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_FULL)
//         ;
//       putreg8(tx_8[i], MG_SPI_TX);
//     }
//     break;

//   case 16:
//     uint16_t *tx_16 = (uint16_t *)data;
//     for (i = 0; i < nwords; i++)
//     {

//       while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_30) && ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31))
//         ;
//       putreg16(tx_16[i], MG_SPI_TX);
//     }
//     break;

//   case 32:
//     const uint32_t *tx_32 = (const uint32_t *)data;
//     for (i = 0; i < nwords; i++)
//     {
//       while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_28) &&
//              (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31) || ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_28_29)))
//         ;
//       putreg32(tx_32[i], MG_SPI_TX);
//     }
//     break;
//   }
//   return (int)i;

// }

// static bool mg_spi_slave_qfull(FAR struct spi_slave_ctrlr_s *ctrlr)
// {
//   struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)ctrlr;

//   /* Check the hardware FIFO status register */
//   uint32_t status = getreg32( MG_SPI_FIFO_STAT);

//   /* Return true if the FULL bit is set, false otherwise */
//   if (status & MG_SPI_TX_FIFO_FULL)
//     {
//       return true;
//     }

//   return false;
// }
// static void mg_spi_slave_qflush(FAR struct spi_slave_ctrlr_s *ctrlr)
// {
//   switch (priv->nbits)
//     {
//     case 8:
//       printf("inside");
//       uint8_t *rx_8 = (const uint8_t *)rxbuffer;
//       for (i = 0; i < nwords; i++)
//       {
//         while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_RX_FIFO_EMPTY)
//           ;
//         rx_8[i] = getreg8(MG_SPI_RX);
//       }
//       break;

//     case 16:
//       uint16_t *rx_16 = (const uint16_t *)rxbuffer;
//       for (i = 0; i < nwords; i++)
//       {
//         while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_2_3)
//           ;
//         rx_16[i] = getreg16(MG_SPI_RX);
//       }
//       break;

//     case 32:
//       uint32_t *rx_32 = (const uint32_t *)rxbuffer;
//       for (i = 0; i < nwords; i++)
//       {
//         while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_4_7)
//           ;
//         rx_32[i] = getreg32(MG_SPI_RX);
//       }
//       break;
//     }
//   return 0;
// }

 
static int mg_spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  FAR struct mg_spi_priv_s *priv = (FAR struct mg_spi_priv_s *)dev;
  uint8_t regval;

  /* Read the current state of the NCS Control Register */
  regval = getreg8(MG_SPI_NCS_CTRL);

  /* Bit 2 (FORCE_ACTIVE) is the standard NuttX way to request Manual CS control */
  if (features & HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER)
    {
  
      regval |= (1 << 0);
    
    }
  else
    {
     
      regval &= ~(1 << 0);
      
      printf("SPI: Hardware NCS Control Enabled\n");
    }

  /* Write the final value back to the register */
  putreg8(regval, MG_SPI_NCS_CTRL);

  return OK;
}
static void mg_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  FAR struct mg_spi_priv_s *priv = (FAR struct mg_spi_priv_s *)dev;
  
  uint8_t regval = getreg8(MG_SPI_NCS_CTRL);
  
  if (selected)
    {
      // SPI Select = Active Low (usually 0)
      regval &= ~(1 << 1); 
    }
  else
    {
      // SPI De-select = High (1)
      regval |= (1 << 1);
    }

putreg8(regval, MG_SPI_NCS_CTRL);
  
  // Debug: Use %d for bool and show the final hex regval
  printf("SPI Select: %s | Reg: 0x%02x\n", selected ? "LOW" : "HIGH", regval);
}
// static int mg_spi_hwfeatures(FAR struct spi_dev_s *dev, spi_hwfeatures_t features)
// {
//   FAR struct mg_spi_priv_s *priv = (FAR struct mg_spi_priv_s *)dev;
//   uint8_t regval = 0;

//   /* 1. Check if user wants Software NCS Control */
//   if (features & SPI_HWFEAT_CSKEEP)
//     {
//       /* Set NCS_SELECT (Bit 0) to 1 for Software Mode */
//       regval |= (1 << 0);
      
//       /* You can also set the initial state of NCS_SW (Bit 1) 
//        * Usually, we start with it High (Deselected = 1) */
//       regval |= (1 << 1);
//     }
//   else
//     {
//       /* Hardware NCS mode (Bit 0 = 0) */
//       regval &= ~(1 << 0);
//     }

//   /* 2. Write to the 8-bit NCS_CTRL register */
//   putreg8(regval, priv->hw_base + MG_SPI_NCS_CTRL_OFFSET);

//   return OK;
// }
static void mg_spi_hwinit(struct mg_spi_priv_s *priv)
{

  /* Master mode: clear SLAVE_MODE bit */

  modifyreg32(MG_SPI_CONTROL, MG_SPI_MODE , MG_SPI_MASTER_MODE);

  /* Enable master outputs:
   *  SCLK, NCS, MOSI = output
   *  MISO = input
   */
  modifyreg32(MG_SPI_CONTROL,
              0,
              MG_SPI_SCLK_OUTEN_MASK |
                  MG_SPI_NCS_OUTEN_MASK |
                  MG_SPI_MOSI_OUTEN_MASK);

  modifyreg32(MG_SPI_CONTROL, MG_SPI_MISO_OUTEN_MASK, 0);
}

struct spi_dev_s *mg_spibus_initialize(int bus)
{

  // struct mg_spi_priv_s *priv;
  struct mg_spi_priv_s *priv = NULL; // Initialize to NULL

  switch (bus)
  {
  case 0:
    priv = &g_mg_spi0_priv;
    break;
  case 1:
    priv = &g_mg_spi1_priv;
    break;
  case 2:
    priv = &g_mg_spi2_priv;
    break;
  case 3:
    priv = &g_mg_spi3_priv;
    break;
  default:
    NULL;
  }

  /* One-time hardware init */
  mg_spi_hwinit(priv);

  priv->enabled = true;

  return &priv->spi;
}

static int mg_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  if (lock)
  {
    nxmutex_lock(&priv->lock);
  }
  else
  {
    nxmutex_unlock(&priv->lock);
  }

  return OK;
}

static uint32_t mg_spi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;
  uint32_t divider;

  DEBUGASSERT(frequency > 0);

  priv->frequency = frequency;
  divider = (CLOCK_FREQUENCY_BASE / frequency) - 1U;
  priv->actual = CLOCK_FREQUENCY_BASE / (divider + 1U);

  modifyreg32(MG_SPI_CLK_CTRL, SPI_CLK_PRESCALAR_MASK, divider & SPI_CLK_PRESCALAR_MASK);

  spiinfo("frequency=%u, actual=%u\n", priv->frequency, priv->actual);

  return priv->actual;
}

static void mg_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);

  // spi mode

  switch (mode)
  {
  case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
    modifyreg32(MG_SPI_CLK_CTRL, SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK, 0);
    break;

  case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
    modifyreg32(MG_SPI_CLK_CTRL, SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK, SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK);
    break;
  }

  priv->mode = mode;

  /* SPS bit ensures the slave select remains asserted between frames,
   * applicable to SPIDEV_MODEx:s.
   */
}

static int mg_spi_setdelay(struct spi_dev_s *dev,
                           uint32_t startdelay,
                           uint32_t stopdelay,
                           uint32_t csdelay,
                           uint32_t ifdelay)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  uint32_t setup;
  uint32_t hold;

  setup = startdelay;
  hold = stopdelay;

  if (setup > 0xff)
    setup = 0x0;
  if (hold > 0xff)
    hold = 0x0;

  modifyreg32(MG_SPI_CLK_CTRL,
              (0xff << 16) | (0xff << 24),
              (setup << 16) | (hold << 24));

  return OK;
}

static void mg_spi_setbits(struct spi_dev_s *dev, int nbits)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  DEBUGASSERT(nbits == 8 || nbits == 16 || nbits == 32);
  modifyreg32(MG_SPI_CONTROL, MG_SPI_TOTAL_BIT_TX_MASK | MG_SPI_TOTAL_BIT_RX_MASK, nbits << MG_SPI_TOTAL_BIT_TX_SHIFT | nbits << MG_SPI_TOTAL_BIT_RX_SHIFT);

  priv->nbits = nbits;
}

static uint8_t mg_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}

static void mg_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                            void *rxbuffer, size_t nwords)
{
  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  // printf(" nwords %d",nwords);
  uint8_t comm_mode = (txbuffer && rxbuffer) ? 3 : (rxbuffer) ? 1
                                                              : 0;
  modifyreg32(MG_SPI_CONTROL, MG_SPI_COMM_MODE_MASK, (uint32_t)comm_mode << MG_SPI_COMM_MODE_SHIFT);

  size_t i;
  modifyreg32(MG_SPI_CONTROL, 0, MG_SPI_ENABLE_MASK);
  switch (comm_mode)
  {
  case (0):
    switch (priv->nbits)
    {

    case 8:
      const uint8_t *tx_8 = (const uint8_t *)txbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_FULL)
          ;
        putreg8(tx_8[i], MG_SPI_TX);
      }
      break;

    case 16:
      uint16_t *tx_16 = (uint16_t *)txbuffer;
      for (i = 0; i < nwords; i++)
      {

        while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_30) && ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31))
          ;
        putreg16(tx_16[i], MG_SPI_TX);
      }
      break;

    case 32:
      const uint32_t *tx_32 = (const uint32_t *)txbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_28) &&
               (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31) || ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_28_29)))
          ;
        putreg32(tx_32[i], MG_SPI_TX);
      }
      break;
    }
    modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0);
    break;

  case (1):
    switch (priv->nbits)
    {
    case 8:
      printf("inside");
      uint8_t *rx_8 = (const uint8_t *)rxbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_RX_FIFO_EMPTY)
          ;
        rx_8[i] = getreg8(MG_SPI_RX);
      }
      break;

    case 16:
      uint16_t *rx_16 = (const uint16_t *)rxbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_2_3)
          ;
        rx_16[i] = getreg16(MG_SPI_RX);
      }
      break;

    case 32:
      uint32_t *rx_32 = (const uint32_t *)rxbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_4_7)
          ;
        rx_32[i] = getreg32(MG_SPI_RX);
      }
      break;
    }
    modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0);
    break;

  case 3:

    switch (priv->nbits)
    {

    case 8:

      const uint8_t *tx_f_8 = (const uint8_t *)txbuffer;
      uint8_t *rx_f_8 = (const uint8_t *)rxbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_FULL)
          ;
        putreg8(tx_f_8[i], MG_SPI_TX);
        while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_RX_FIFO_EMPTY)
          ;
        rx_f_8[i] = getreg8(MG_SPI_RX);
      }

      break;

    case 16:

      const uint16_t *tx_f_16 = (const uint16_t *)txbuffer;
      uint16_t *rx_f_16 = (const uint16_t *)rxbuffer;
      for ( i = 0; i < nwords; i++)
      {
        while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_30) && ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31))
          ;
        putreg16(tx_f_16[i], MG_SPI_TX);
        while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_2_3)
          ;
        rx_f_16[i] = getreg16(MG_SPI_RX);
      }

      break;

    case 32:
      printf("caseok");
      const uint32_t *tx_f_32 = (const uint32_t *)txbuffer;
      uint32_t *rx_f_32 = (const uint32_t *)rxbuffer;
      for (i = 0; i < nwords; i++)
      {
        while (!(getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_28) &&
               (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_30_31) || ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK) >> 3 == MG_SPI_RX_DEPTH_28_29)))
          ;
        putreg32(tx_f_32[i], MG_SPI_TX);
        while (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK) >> 6) < MG_SPI_RX_DEPTH_4_7)
          ;
        rx_f_32[i] = getreg32(MG_SPI_RX);
      }

      break;
      //   }

      break;
    }

    modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0);
  }
}

static uint32_t mg_spi_recv(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t rx;

  mg_spi_exchange(dev, &wd, &rx, 1);

  return rx;
}

static uint32_t mg_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t tx;

  mg_spi_exchange(dev, &tx, &wd, 1);

  return tx;
}
// static const struct spi_ops_s g_mg_spi_ops =
// {
//   .lock       = mg_spi_lock,
//   .setfrequency = mg_spi_setfrequency,
//   .setmode    = mg_spi_setmode,
//   .setbits    = mg_spi_setbits,
//   .status     = mg_spi_status,
// #ifdef CONFIG_SPI_EXCHANGE
//   .exchange   = mg_spi_exchange,
// #else
//   .sndblock   = mg_spi_send,
//   .recvblock  = mg_spi_recv,
// #endif
// };
