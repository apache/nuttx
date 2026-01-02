


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



struct mg_spi_priv_s
{
  struct spi_dev_s spi;     /* MUST be first */

  uintptr_t        hw_base;    /* SPI base address */

  bool             enabled;

  uint32_t         frequency;
  uint32_t         actual;

  enum spi_mode_e  mode;
  uint8_t          nbits;

  uint32_t         devid;   /* CS device ID */

  mutex_t          lock;    /* SPI bus lock */
};


static int      mg_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t mg_spi_setfrequency(FAR struct spi_dev_s *dev,
                                    uint32_t frequency);
static void     mg_spi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void     mg_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint8_t  mg_spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
static void     mg_spi_exchange(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer,
                                FAR void *rxbuffer,
                                size_t nwords);
                                static void mg_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected);
struct mg_spi_config_s
{
  uint32_t        clk_freq;  /* SPI clock frequency */
  enum spi_mode_e mode;      /* SPI default mode */
  bool            use_irq;   /* Use DMA */
};

static const struct spi_ops_s g_mg_spi_ops =
{
  .lock         = mg_spi_lock,
  .select       = mg_spi_select,      /* ADD THIS LINE */
  .setfrequency = mg_spi_setfrequency,
  .setmode      = mg_spi_setmode,
  .setbits      = mg_spi_setbits,
  .status       = mg_spi_status,
  .exchange     = mg_spi_exchange,
};



static struct mg_spi_priv_s g_mg_spi0_priv =
{
  .spi =
    {
      .ops = &g_mg_spi_ops,
    },
  .hw_base   = MG_SPI0_BASE,
  .enabled   = false,
  .lock      = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi1_priv =
{
  .spi =
    {
      .ops = &g_mg_spi_ops,
    },
  .hw_base   = MG_SPI1_BASE,
  .enabled   = false,
  .lock      = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi2_priv =
{
  .spi =
    {
      .ops = &g_mg_spi_ops,
    },
  .hw_base   = MG_SPI2_BASE,
  .enabled   = false,
  .lock      = NXMUTEX_INITIALIZER,
};

static struct mg_spi_priv_s g_mg_spi3_priv =
{
  .spi =
    {
      .ops = &g_mg_spi_ops,
    },
  .hw_base   = MG_SPI3_BASE,
  .enabled   = false,
  .lock      = NXMUTEX_INITIALIZER,
};
static void mg_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  /* This is where you would normally pull the CS pin LOW (true) or HIGH (false) */
  /* If your hardware manages CS automatically via the 'devid', you can leave this empty */

  
}
static void mg_spi_hwinit(struct mg_spi_priv_s *priv)
{


  /* Master mode: clear SLAVE_MODE bit */
  modifyreg32(MG_SPI_CONTROL, MG_SPI_MODE, 0);

  /* Enable master outputs:
   *  SCLK, NCS, MOSI = output
   *  MISO = input
   */
  modifyreg32(MG_SPI_CONTROL,
              0,
              MG_SPI_SCLK_OUTEN_MASK |
              MG_SPI_NCS_OUTEN_MASK  |
              MG_SPI_MOSI_OUTEN_MASK);

  modifyreg32(MG_SPI_CONTROL, MG_SPI_MISO_OUTEN_MASK, 0);
}

struct spi_dev_s *mg_spibus_initialize(int bus)
{
  
  struct mg_spi_priv_s *priv;

  switch (bus)
    {
      case 0:priv = &g_mg_spi0_priv;break;
      case 1:priv = &g_mg_spi1_priv;break;
      case 2:priv = &g_mg_spi2_priv;break;
      case 3:priv = &g_mg_spi3_priv;break;
      default:NULL;

       
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
  divider = (CLOCK_FREQUENCY_BASE / frequency) -1U;
  priv->actual = CLOCK_FREQUENCY_BASE / (divider + 1U);
  
     modifyreg32(MG_SPI_CLK_CTRL,SPI_CLK_PRESCALAR_MASK,divider & SPI_CLK_PRESCALAR_MASK);
  
  spiinfo("frequency=%u, actual=%u\n", priv->frequency, priv->actual);

  
  return priv->actual;
}


static void mg_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
 
  
  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;

  spiinfo("mode=%d\n", mode);




  //spi mode 

  switch (mode)
    {
      case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
        modifyreg32(MG_SPI_CLK_CTRL,SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK,0);
        break;

      case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
        modifyreg32(MG_SPI_CLK_CTRL,SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK,SPI_CLK_POLARITY_MASK | SPI_CLK_PHASE_MASK);
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

  setup = startdelay ;
  hold  = stopdelay  ;

  if (setup > 0xff) setup = 0x0;
  if (hold  > 0xff) hold  = 0x0;

  modifyreg32(MG_SPI_CLK_CTRL,
              (0xff << 16) | (0xff << 24),
              (setup << 16) | (hold << 24));

  return OK;
}


static void mg_spi_setbits(struct spi_dev_s *dev, int nbits)
{

  struct mg_spi_priv_s *priv = (struct mg_spi_priv_s *)dev;
  
  DEBUGASSERT(nbits == 8 || nbits == 16 || nbits == 32);
  modifyreg32(MG_SPI_CONTROL, MG_SPI_TOTAL_BIT_TX_MASK|MG_SPI_TOTAL_BIT_RX_MASK, nbits<<MG_SPI_TOTAL_BIT_TX_SHIFT| nbits<<MG_SPI_TOTAL_BIT_RX_SHIFT);

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
  uint8_t comm_mode = (txbuffer && rxbuffer) ? 3 : (rxbuffer) ? 1 : 0;
  modifyreg32(MG_SPI_CONTROL,MG_SPI_COMM_MODE_MASK,(uint32_t)comm_mode << MG_SPI_COMM_MODE_SHIFT);
  

  size_t i;
  modifyreg32(MG_SPI_CONTROL,0, MG_SPI_ENABLE_MASK);
  switch ( comm_mode ){
    case (0):
          switch (priv-> nbits){

            case 8:
              const uint8_t *tx_8= (const uint8_t *)txbuffer;
              for (i = 0; i < nwords; i++){
                while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_FULL);
                putreg8(tx_8[i], MG_SPI_TX);
              }
              break;
            
            case 16:
               uint16_t *tx_16= (uint16_t *)txbuffer;
              for (i = 0; i < nwords; i++){
               
                while (!(getreg32(MG_SPI_FIFO_STAT) &  MG_SPI_TX_FIFO_30)\
                      && ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 ==MG_SPI_RX_DEPTH_30_31));
                putreg16(tx_16[i], MG_SPI_TX);
              }
              break;
              

            case 32:
              const uint32_t *tx_32= (const uint32_t *)txbuffer;
              for (i = 0; i < nwords; i++){
                while (!(getreg32(MG_SPI_FIFO_STAT) &  MG_SPI_TX_FIFO_28) &&\
                      (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 ==MG_SPI_RX_DEPTH_30_31)\
                      ||((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 ==MG_SPI_RX_DEPTH_28_29)));
                putreg32(tx_32[i], MG_SPI_TX);
              }
              break;  
            }
    modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0);  
    break;


    case (1):
      switch (priv-> nbits){
        case 8:
        printf("inside");
          uint8_t *rx_8 = (const uint8_t *)rxbuffer;
          for (i = 0; i < nwords; i++){
            while  (getreg32(MG_SPI_FIFO_STAT ) & MG_SPI_RX_FIFO_EMPTY);
            rx_8[i] = getreg8(MG_SPI_RX);
          }
          break;
        
          case 16:
            uint16_t *rx_16 = (const uint16_t *)rxbuffer;
            for (i = 0; i < nwords; i++){
              while  (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK)>>6)<MG_SPI_RX_DEPTH_2_3);
              rx_16[i] = getreg16(MG_SPI_RX);
            }
            break;
          
          case 32:
            uint32_t *rx_32 = (const uint32_t *)rxbuffer;
            for (i = 0; i < nwords; i++){
              while  (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK)>>6)< MG_SPI_RX_DEPTH_4_7);
              rx_32[i] = getreg32(MG_SPI_RX);
            }
            break;
          }
       modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0); 
    break;

      case 3:

        switch (priv-> nbits){

          case 8:

            const uint8_t *tx_f_8= (const uint8_t *)txbuffer;
            uint8_t *rx_f_8 = (const uint8_t *)rxbuffer;
            for (i = 0; i < nwords; i++){
              while (getreg32(MG_SPI_FIFO_STAT) & MG_SPI_TX_FIFO_FULL);
              putreg8(tx_f_8[i], MG_SPI_TX);
              while  (getreg32(MG_SPI_FIFO_STAT ) & MG_SPI_RX_FIFO_EMPTY);
              rx_f_8[i] = getreg8(MG_SPI_RX); 
            }
           
            break;

          case 16:
        
            const uint16_t *tx_f_16= (const uint16_t *)txbuffer;
            uint16_t *rx_f_16 = (const uint16_t *)rxbuffer;
            for (int i = 0; i < nwords; i++){
               while (!(getreg32(MG_SPI_FIFO_STAT) &  MG_SPI_TX_FIFO_30)\
                      && ((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 ==MG_SPI_RX_DEPTH_30_31));
              putreg16(tx_f_16[i], MG_SPI_TX);
              while  (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK)>>6)<MG_SPI_RX_DEPTH_2_3);
              rx_f_16[i] = getreg16(MG_SPI_RX);
            }
          
            break;
          
          case 32:
            printf("caseok");
            const uint32_t *tx_f_32= (const uint32_t *)txbuffer;
            uint32_t *rx_f_32 = (const uint32_t *)rxbuffer;
            for (int i = 0; i < nwords; i++){
              while (!(getreg32(MG_SPI_FIFO_STAT) &  MG_SPI_TX_FIFO_28) &&\
                      (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 == MG_SPI_RX_DEPTH_30_31)\
                      ||((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_TX_DEPTH_MASK)>>3 == MG_SPI_RX_DEPTH_28_29)));
              putreg32(tx_f_32[i], MG_SPI_TX);
              while  (((getreg16(MG_SPI_COMM_STAT) & MG_SPI_COMM_STATUS_RX_DEPTH_MASK)>>6)< MG_SPI_RX_DEPTH_4_7);
              rx_f_32[i] = getreg32(MG_SPI_RX);
            }
          
            break;
        //   }

     break;
    }

     modifyreg32(MG_SPI_CONTROL, MG_SPI_ENABLE_MASK, 0);  
  }}



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

