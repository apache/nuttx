/****************************************************************************
 * arch/arm/src/imx1/imx_cspi.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_CSPI_H
#define __ARCH_ARM_SRC_IMX1_IMX_CSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <nuttx/spi/spi.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CSPI Register Offsets ****************************************************/

#define CSPI_RXD_OFFSET          0x0000 /* Receive Data Register */
#define CSPI_TXD_OFFSET          0x0004 /* Transmit Data Register */
#define CSPI_CTRL_OFFSET         0x0008 /* Control Register */
#define CSPI_INTCS_OFFSET        0x000c /* Interrupt Control/Status Register */
#define CSPI_TEST_OFFSET         0x0010 /* Test Register */
#define CSPI_SPCR_OFFSET         0x0014 /* Sample Period Control Register */
#define CSPI_DMA_OFFSET          0x0018 /* DMA Control Register */
#define CSPI_RESET_OFFSET        0x001c /* Soft Reset Register */

/* CSPI Register Addresses **************************************************/

/* CSPI1 */

#define IMX_CSPI1_RXD            (IMX_CSPI1_VBASE + CSPI_RXD_OFFSET)
#define IMX_CSPI1_TXD            (IMX_CSPI1_VBASE + CSPI_TXD_OFFSET)
#define IMX_CSPI1_CTRL           (IMX_CSPI1_VBASE + CSPI_CTRL_OFFSET)
#define IMX_CSPI1_INTCS          (IMX_CSPI1_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI1_SPITEST        (IMX_CSPI1_VBASE + CSPI_TEST_OFFSET)
#define IMX_CSPI1_SPISPCR        (IMX_CSPI1_VBASE + CSPI_SPCR_OFFSET)
#define IMX_CSPI1_SPIDMA         (IMX_CSPI1_VBASE + CSPI_DMA_OFFSET)
#define IMX_CSPI1_SPIRESET       (IMX_CSPI1_VBASE + CSPI_RESET_OFFSET)

/* CSPI2 */

#define IMX_CSPI2_RXD            (IMX_CSPI2_VBASE + CSPI_RXD_OFFSET)
#define IMX_CSPI2_TXD            (IMX_CSPI2_VBASE + CSPI_TXD_OFFSET)
#define IMX_CSPI2_CTRL           (IMX_CSPI2_VBASE + CSPI_CTRL_OFFSET)
#define IMX_CSPI2_INTCS          (IMX_CSPI2_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI2_SPITEST        (IMX_CSPI2_VBASE + CSPI_TEST_OFFSET)
#define IMX_CSPI2_SPISPCR        (IMX_CSPI2_VBASE + CSPI_SPCR_OFFSET)
#define IMX_CSPI2_SPIDMA         (IMX_CSPI2_VBASE + CSPI_DMA_OFFSET)
#define IMX_CSPI2_SPIRESET       (IMX_CSPI2_VBASE + CSPI_RESET_OFFSET)

/* CSPI Register Bit Definitions ********************************************/

/* CSPI Control Register */

#define CSPI_CTRL_DATARATE_SHIFT 13
#define CSPI_CTRL_DATARATE_MASK    (7 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV4             (0 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV8             (1 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV16            (2 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV32            (3 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV64            (4 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV128           (5 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV256           (6 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV512           (7 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DRCTL_SHIFT      11
#define CSPI_CTRL_DRCTL_MASK       (3 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_IGNRDY     (0 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_FALLING    (1 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_ACTVLOW    (2 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_MODE             (1 << 10)
#define CSPI_CTRL_SPIEN            (1 << 9)
#define CSPI_CTRL_XCH              (1 << 8)
#define CSPI_CTRL_SSPOL            (1 << 7)
#define CSPI_CTRL_SSCTL            (1 << 6)
#define CSPI_CTRL_PHA              (1 << 5)
#define CSPI_CTRL_POL              (1 << 4)
#define CSPI_CTRL_BITCOUNT_SHIFT   0
#define CSPI_CTRL_BITCOUNT_MASK    (15 << CSPI_CTRL_BITCOUNT_SHIFT)

/* CSPI Interrupt Control/Status Register */

#define CSPI_INTCS_TE              (1 << 0)  /* Bit  0: TXFIFO Empty Status */
#define CSPI_INTCS_TH              (1 << 1)  /* Bit  1: TXFIFO Half Status */
#define CSPI_INTCS_TF              (1 << 2)  /* Bit  2: TXFIFO Full Status */
#define CSPI_INTCS_RR              (1 << 3)  /* Bit  3: RXFIFO Data Ready Status */
#define CSPI_INTCS_RH              (1 << 4)  /* Bit  4: RXFIFO Half Status */
#define CSPI_INTCS_RF              (1 << 5)  /* Bit  5: RXFIFO Full Status */
#define CSPI_INTCS_RO              (1 << 6)  /* Bit  6: RXFIFO Overflow */
#define CSPI_INTCS_BO              (1 << 7)  /* Bit  7: Bit Count Overflow */
#define CSPI_INTCS_TEEN            (1 << 8)  /* Bit  8: TXFIFO Empty Interrupt Enable */
#define CSPI_INTCS_THEN            (1 << 9)  /* Bit  9: TXFIFO Half Interrupt Enable */
#define CSPI_INTCS_TFEN            (1 << 10) /* Bit 10: TXFIFO Full Interrupt Enable */
#define CSPI_INTCS_RREN            (1 << 11) /* Bit 11: RXFIFO Data Ready Interrupt Enable */
#define CSPI_INTCS_RHEN            (1 << 12) /* Bit 12: RXFIFO Half Interrupt Enable */
#define CSPI_INTCS_RFEN            (1 << 13) /* Bit 13: RXFIFO Full Interrupt Enable */
#define CSPI_INTCS_ROEN            (1 << 14) /* BIT 14: RXFIFO Overflow Interrupt Enable */
#define CSPI_INTCS_BOEN            (1 << 15) /* Bit 15: Bit Count Overflow Interrupt Enable */

#define CSPI_INTCS_ALLINTS         0x0000ff00

/* CSPI Sample Period Control Register */

#define CSPI_SPCR_WAIT_SHIFT       0
#define CSPI_SPCR_WAIT_MASK        (0x7ff << CSPI_SPCR_WAIT_SHIFT)
#define CSPI_SPCR_CSRC             (1 << 15) /* Bit 15: 1:32768 or 32 kHz clock source */

/* CSPI DMA Control Register */

#define CSPI_DMA_RHDMA             (1 <<  4) /* Bit  4: RXFIFO Half Status */
#define CSPI_DMA_RFDMA             (1 <<  5) /* Bit  5: RXFIFO Full Status */
#define CSPI_DMA_TEDMA             (1 <<  6) /* Bit  6: TXFIFO Empty Status */
#define CSPI_DMA_THDMA             (1 <<  7) /* Bit  7: TXFIFO Half Status */
#define CSPI_DMA_RHDEN             (1 << 12) /* Bit 12: Enable RXFIFO Half DMA Request */
#define CSPI_DMA_RFDEN             (1 << 13) /* Bit 13: Enables RXFIFO Full DMA Request */
#define CSPI_DMA_TEDEN             (1 << 14) /* Bit 14: Enable TXFIFO Empty DMA Request */
#define CSPI_DMA_THDEN             (1 << 15) /* Bit 15: Enable TXFIFO Half DMA Request */

/* Soft Reset Register */

#define CSPI_RESET_START           (1 << 0)  /* Bit  0: Execute soft reset */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif /* __cplusplus */

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: imx_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.
 *   However, If multiple devices on on the bus, then multiple chip selects
 *   will be required.  Therefore, all GPIO chip management is deferred to
 *   board- specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *imx_spibus_initialize(int port);

/****************************************************************************
 * The external functions, imx_spiselect, imx_spistatus, and imx_cmddaa must
 * be provided by board-specific logic.  These are implementations of the
 * select and status methods of the SPI interface defined by struct spi_ops_s
 * (see include/nuttx/spi/spi.h).
 * All other methods (including imx_spibus_initialize()) are provided by
 * common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide imx_spiselect() and imx_spistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide
 *      the imx_spicmddata() function in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to imx_spibus_initialize() in your low level
 *      initialization logic
 *   4. The handle returned by imx_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void imx_spiselect(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spistatus(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spicmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_IMX1_IMX_CSPI_H */
