/****************************************************************************
 * arch/x86/src/qemu/qemu.h
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

#ifndef __ARCH_X86_SRC_QEMU_QEMU_H
#define __ARCH_X86_SRC_QEMU_QEMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spi_dev_s;  /* Forward reference */

/****************************************************************************
 * Name: i486_clockconfig
 *
 * Description:
 *   Called to initialize the i486.  This does whatever setup is needed to
 *   put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void i486_clockconfig(void);

/****************************************************************************
 * Name: i486_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void i486_lowsetup(void);

/****************************************************************************
 * Name: i486_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_QEMU_GPIOIRQ
void i486_gpioirqinitialize(void);
#else
#  define i486_gpioirqinitialize()
#endif

/****************************************************************************
 * Name: i486_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int i486_configgpio(uint16_t cfgset);

/****************************************************************************
 * Name: i486_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void i486_gpiowrite(uint16_t pinset, bool value);

/****************************************************************************
 * Name: i486_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool i486_gpioread(uint16_t pinset);

/****************************************************************************
 * Name: i486_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_QEMU_GPIOIRQ
void i486_gpioirqenable(int irq);
#else
#  define i486_gpioirqenable(irq)
#endif

/****************************************************************************
 * Name: i486_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_QEMU_GPIOIRQ
void i486_gpioirqdisable(int irq);
#else
#  define i486_gpioirqdisable(irq)
#endif

/****************************************************************************
 * Function:  i486_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int i486_dumpgpio(uint16_t pinset, const char *msg);
#else
#  define i486_dumpgpio(p,m)
#endif

/****************************************************************************
 * Name: i486_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *i486_spibus_initialize(int port);

/****************************************************************************
 * Name:  i486_spi/ssp0/ssp1select, i486_spi/ssp0/ssp1status, and
 *        i486_spi/ssp0/ssp1cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including i486_spibus_initialize()) are provided by
 *   common i486 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in i486_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide i486_spi/ssp0/ssp1select() and i486_spi/ssp0/ssp1status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      i486_spi/ssp0/ssp1cmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   3. Add a call to i486_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by i486_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_I486_SPI
void  i486_spiselect(FAR struct spi_dev_s *dev,
                     uint32_t devid, bool selected);
uint8_t i486_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int i486_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_I486_SPI
void spi_flush(FAR struct spi_dev_s *dev);
#endif
#if defined(CONFIG_I486_SSP0) || defined(CONFIG_I486_SSP1)
void ssp_flush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: i486_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
void i486_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: i486_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
DMA_HANDLE i486_dmachannel(void);
#endif

/****************************************************************************
 * Name: i486_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until i486_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
void i486_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: i486_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
int i486_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                    uint32_t srcaddr, uint32_t destaddr, size_t nbytes);
#endif

/****************************************************************************
 * Name: i486_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
int i486_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: i486_dmastop
 *
 * Description:
 *   Cancel the DMA.  After i486_dmastop() is called, the DMA channel is
 *   reset and i486_dmasetup() must be called before i486_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
void i486_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: i486_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
#ifdef CONFIG_DEBUG_DMA
void i486_dmasample(DMA_HANDLE handle, struct i486_dmaregs_s *regs);
#else
#  define i486_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: i486_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_I486_GPDMA
#ifdef CONFIG_DEBUG_DMA
void i486_dmadump(DMA_HANDLE handle, const struct i486_dmaregs_s *regs,
                  const char *msg);
#else
#  define i486_dmadump(handle,regs,msg)
#endif
#endif

/****************************************************************************
 * Name: vector_*
 *
 * Description:
 *   These are the various ISR/IRQ vector address exported from
 *   qemu_vectors.S.  These addresses need to have global scope so that they
 *   can be known to the interrupt initialization logic in qemu_irq.c.
 *
 ****************************************************************************/

void vector_isr0(void);
void vector_isr1(void);
void vector_isr2(void);
void vector_isr3(void);
void vector_isr4(void);
void vector_isr5(void);
void vector_isr6(void);
void vector_isr7(void);
void vector_isr8(void);
void vector_isr9(void);
void vector_isr10(void);
void vector_isr11(void);
void vector_isr12(void);
void vector_isr13(void);
void vector_isr14(void);
void vector_isr15(void);
void vector_isr16(void);
void vector_isr17(void);
void vector_isr18(void);
void vector_isr19(void);
void vector_isr20(void);
void vector_isr21(void);
void vector_isr22(void);
void vector_isr23(void);
void vector_isr24(void);
void vector_isr25(void);
void vector_isr26(void);
void vector_isr27(void);
void vector_isr28(void);
void vector_isr29(void);
void vector_isr30(void);
void vector_isr31(void);
void vector_irq0(void);
void vector_irq1(void);
void vector_irq2(void);
void vector_irq3(void);
void vector_irq4(void);
void vector_irq5(void);
void vector_irq6(void);
void vector_irq7(void);
void vector_irq8(void);
void vector_irq9(void);
void vector_irq10(void);
void vector_irq11(void);
void vector_irq12(void);
void vector_irq13(void);
void vector_irq14(void);
void vector_irq15(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_SRC_QEMU_QEMU_H */
