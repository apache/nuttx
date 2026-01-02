#define MG_SPI_MODE                0

#define MG_SPI_ENABLE_SHIFT        1
#define MG_SPI_ENABLE_MASK         (1U << MG_SPI_ENABLE_SHIFT)

#define MG_SPI_LSB_FIRST_SHIFT     2
#define MG_SPI_LSB_FIRST_MASK      (1U << MG_SPI_LSB_FIRST_SHIFT)

#define MG_SPI_COMM_MODE_SHIFT     4
#define MG_SPI_COMM_MODE_MASK      (0x3U << MG_SPI_COMM_MODE_SHIFT)

#define MG_SPI_TOTAL_BIT_TX_SHIFT  6
#define MG_SPI_TOTAL_BIT_TX_MASK   (0xFFU << MG_SPI_TOTAL_BIT_TX_SHIFT)

#define MG_SPI_TOTAL_BIT_RX_SHIFT  14
#define MG_SPI_TOTAL_BIT_RX_MASK   (0xFFU << MG_SPI_TOTAL_BIT_RX_SHIFT)

#define MG_SPI_SCLK_OUTEN_SHIFT    22
#define MG_SPI_SCLK_OUTEN_MASK     (1U << MG_SPI_SCLK_OUTEN_SHIFT)

#define MG_SPI_NCS_OUTEN_SHIFT     23
#define MG_SPI_NCS_OUTEN_MASK      (1U << MG_SPI_NCS_OUTEN_SHIFT)

#define MG_SPI_MISO_OUTEN_SHIFT    24
#define MG_SPI_MISO_OUTEN_MASK     (1U << MG_SPI_MISO_OUTEN_SHIFT)

#define MG_SPI_MOSI_OUTEN_SHIFT    25
#define MG_SPI_MOSI_OUTEN_MASK     (1U << MG_SPI_MOSI_OUTEN_SHIFT)


//CLK_CTRL_REG

#define CLOCK_FREQUENCY_BASE       35000000u
#define SPI_CLK_POLARITY_SHIFT   0
#define SPI_CLK_POLARITY_MASK    (1<<SPI_CLK_POLARITY_SHIFT)

#define SPI_CLK_PHASE_SHIFT      1
#define SPI_CLK_PHASE_MASK       (1<<SPI_CLK_PHASE_SHIFT)

#define SPI_CLK_PRESCALAR_SHIFT  2
#define SPI_CLK_PRESCALAR_MASK   (0x3FFF << SPI_CLK_PRESCALAR_SHIFT) /* bits [15:2] */

#define SPI_CLK_SETUP_SHIFT      16
#define SPI_CLK_SETUP_MASK       (0xFF << SPI_CLK_SETUP_SHIFT)

#define SPI_CLK_HOLD_SHIFT       24
#define SPI_CLK_HOLD_MASK        (0xFF << SPI_CLK_HOLD_SHIFT)


#define MG_SPI_CONTROL_OFFSET       0x00
#define MG_SPI_CLK_CTRL_OFFSET      0x04
#define MG_SPI_TX_OFFSET            0x08
#define MG_SPI_RX_OFFSET            0x0C
#define MG_SPI_INTR_EN_OFFSET       0x10
#define MG_SPI_FIFO_STAT_OFFSET     0x14
#define MG_SPI_COMM_STAT_OFFSET     0x18

#define MG_SPI_CONTROL      (priv->hw_base + MG_SPI_CONTROL_OFFSET)
#define MG_SPI_CLK_CTRL     (priv->hw_base + MG_SPI_CLK_CTRL_OFFSET)
#define MG_SPI_TX           (priv->hw_base + MG_SPI_TX_OFFSET)
#define MG_SPI_RX           (priv->hw_base + MG_SPI_RX_OFFSET)
#define MG_SPI_INTR_EN      (priv->hw_base + MG_SPI_INTR_EN_OFFSET)
#define MG_SPI_FIFO_STAT    (priv->hw_base + MG_SPI_FIFO_STAT_OFFSET)
#define MG_SPI_COMM_STAT    (priv->hw_base + MG_SPI_COMM_STAT_OFFSET)

/* TX FIFO status bits */

#define MG_SPI_TX_FIFO_EMPTY      (1u << 0)
#define MG_SPI_TX_FIFO_DUAL       (1u << 1)  /* 2 entries */
#define MG_SPI_TX_FIFO_QUAD       (1u << 2)  /* 4 entries */
#define MG_SPI_TX_FIFO_OCTAL      (1u << 3)  /* 8 entries */
#define MG_SPI_TX_FIFO_HALF       (1u << 4)  /* 16 entries */
#define MG_SPI_TX_FIFO_24         (1u << 5)  /* 24 entries */
#define MG_SPI_TX_FIFO_28         (1u << 6)  /* 28 entries */
#define MG_SPI_TX_FIFO_30         (1u << 7)  /* 30 entries */
#define MG_SPI_TX_FIFO_FULL       (1u << 8)  /* 32 entries */


/* RX FIFO status bits */

#define MG_SPI_RX_FIFO_EMPTY      (1u << 9)
#define MG_SPI_RX_FIFO_DUAL       (1u << 10) /* 2 entries */
#define MG_SPI_RX_FIFO_QUAD       (1u << 11) /* 4 entries */
#define MG_SPI_RX_FIFO_OCTAL      (1u << 12) /* 8 entries */
#define MG_SPI_RX_FIFO_HALF       (1u << 13) /* 16 entries */
#define MG_SPI_RX_FIFO_24         (1u << 14) /* 24 entries */
#define MG_SPI_RX_FIFO_28         (1u << 15) /* 28 entries */
#define MG_SPI_RX_FIFO_30         (1u << 16) /* 30 entries */
#define MG_SPI_RX_FIFO_FULL       (1u << 17) /* 32 entries */

#define MG_SPI_COMM_STATUS_BUSY          (1 << 0)
#define MG_SPI_COMM_STATUS_TX_STARTED    (1 << 1)
#define MG_SPI_COMM_STATUS_RX_STARTED    (1 << 2)
#define MG_SPI_COMM_STATUS_TX_DEPTH_MASK (0x7 << 3)
#define MG_SPI_COMM_STATUS_RX_DEPTH_MASK (0x7 << 6)
#define MG_SPI_COMM_STATUS_OVERRUN       (1 << 9)

#define MG_SPI_TX_DEPTH_0_1    0x0
#define MG_SPI_TX_DEPTH_2_3    0x1
#define MG_SPI_TX_DEPTH_4_7    0x2
#define MG_SPI_TX_DEPTH_8_15   0x3
#define MG_SPI_TX_DEPTH_16_23  0x4
#define MG_SPI_TX_DEPTH_24_27  0x5
#define MG_SPI_TX_DEPTH_28_29  0x6
#define MG_SPI_TX_DEPTH_30_31  0x7

#define MG_SPI_RX_DEPTH_0_1    0x0
#define MG_SPI_RX_DEPTH_2_3    0x1
#define MG_SPI_RX_DEPTH_4_7    0x2
#define MG_SPI_RX_DEPTH_8_15   0x3
#define MG_SPI_RX_DEPTH_16_23  0x4
#define MG_SPI_RX_DEPTH_24_27  0x5
#define MG_SPI_RX_DEPTH_28_29  0x6
#define MG_SPI_RX_DEPTH_30_31  0x7


#ifndef __ARCH_RISCV_SRC_CHIP_MINDGROVE_SPI_H
#define __ARCH_RISCV_SRC_CHIP_MINDGROVE_SPI_H

#include <nuttx/spi/spi.h>

struct spi_dev_s *mg_spibus_initialize(int bus);

#endif

// // Function declaration
// struct spi_dev_s *mg_spibus_initialize(int bus);