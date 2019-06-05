/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd5602_memorymap.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_MEMORYMAP_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_ADSP_BASE           0x4c000000
#define CXD56_SYS_MIRROR         0x04000000

#define CXD56_SYS_RAM_BASE        0x05000000
#define CXD56_GDSP_RAM_BASE       0x09000000
#define CXD56_ADSP_RAM_BASE       0x0d000000
#define CXD56_RAM_BASE            0x0d000000
#define CXD56_RAM_SIZE            0x00180000
#define CXD56_ARM_BASE            0xe0000000
#define CXD56_TIMER_BASE          0xe0043000
#define CXD56_WDOG_BASE           0xe0044000
#define CXD56_INTC_BASE           0xe0045000

#define CXD56_SWINT_BASE          0x4600c000
#define CXD56_CPUFIFO_BASE        0x4600c400
#define CXD56_SPH_BASE            0x4600c800

/* Peripheral and system configuration */

#define CXD56_ROM_BASE            (CXD56_SYS_MIRROR + 0x00000000)
#define CXD56_TOPREG_BASE         (CXD56_SYS_MIRROR + 0x00100000)
#define CXD56_TOPREG_SUB_BASE     (CXD56_SYS_MIRROR + 0x00103000)
#define CXD56_PMU_SUB_BASE        (CXD56_SYS_MIRROR + 0x00106000)
#define CXD56_FREQDISC_BASE       (CXD56_SYS_MIRROR + 0x00107000)
#define CXD56_RTC0_BASE           (CXD56_SYS_MIRROR + 0x00108000)
#define CXD56_RTC1_BASE           (CXD56_SYS_MIRROR + 0x00109000)
#define CXD56_TDC_BASE            (CXD56_SYS_MIRROR + 0x0010B000)

/* reserved 0x0010c000 - 0x00110fff */

#define CXD56_SPIFLASH_BASE       (CXD56_SYS_MIRROR + 0x00110000)

/* reserved 0x00111000 - 0x0011ffff */

#define CXD56_DMAC0_BASE          (CXD56_SYS_MIRROR + 0x00120000) /* SDMAC */
#define CXD56_DMAC1_BASE          (CXD56_SYS_MIRROR + 0x00121000) /* HDMAC */
#define CXD56_DMAC2_BASE          (CXD56_SYS_MIRROR + 0x00122000) /* SYDMAC */
#define CXD56_DMAC3_BASE          (CXD56_SYS_MIRROR + 0x00123000) /* SYSUB */

/* reserved 0x00124000 - 0x0017ffff */

#define CXD56_SCU_FIFO_REG_BASE   (CXD56_SYS_MIRROR + 0x00180000)
#define CXD56_SCU_FIFO_4K_BASE    (CXD56_SYS_MIRROR + 0x00182000)
#define CXD56_SCU_FIFO_8K_BASE    (CXD56_SYS_MIRROR + 0x00183000)
#define CXD56_SCU_FIFO_32K_BASE   (CXD56_SYS_MIRROR + 0x00185000)
#define CXD56_SCU_SPI_BASE        (CXD56_SYS_MIRROR + 0x0018d000)
#define CXD56_SCU_I2C0_BASE       (CXD56_SYS_MIRROR + 0x0018d400)
#define CXD56_SCU_I2C1_BASE       (CXD56_SYS_MIRROR + 0x0018d800)
#define CXD56_SCU_ADCIF_BASE      (CXD56_SYS_MIRROR + 0x0018dc00)
#define CXD56_SCU_SEQ_IRAM_BASE   (CXD56_SYS_MIRROR + 0x00190000)
#define CXD56_SCU_SEQ_IRAM_MIRROR (CXD56_SYS_MIRROR + 0x00192000)
#define CXD56_SCU_SEQ_DRAM_BASE   (CXD56_SYS_MIRROR + 0x00194000)
#define CXD56_SCU_SEQ_DRAM_MIRROR (CXD56_SYS_MIRROR + 0x00194800)
#define CXD56_SCU_BASE            (CXD56_SYS_MIRROR + 0x00195000)
#define CXD56_UART0_BASE          (CXD56_SYS_MIRROR + 0x001a9000)

#define CXD56_I2CM_BASE           (CXD56_SYS_MIRROR + 0x001aa000)
#define CXD56_SPIM_BASE           (CXD56_SYS_MIRROR + 0x001ab000)
#define CXD56_UART1_BASE          (CXD56_SYS_MIRROR + 0x001ac000)

#define CXD56_CPU_BASE            (CXD56_ADSP_BASE  + 0x02002000)
#define CXD56_CRG_BASE            (CXD56_ADSP_BASE  + 0x02011000)
#define CXD56_ADR_CONV_BASE       (CXD56_ADSP_BASE  + 0x02012000)
#define CXD56_EXCCONF_BASE        (CXD56_ADSP_BASE  + 0x02013000)
#define CXD56_CISIF_BASE          (CXD56_ADSP_BASE  + 0x02100000)
#define CXD56_GE2D_BASE           (CXD56_ADSP_BASE  + 0x02101000)
#define CXD56_ROT_BASE            (CXD56_ADSP_BASE  + 0x02101400)
#define CXD56_UART2_BASE          (CXD56_ADSP_BASE  + 0x02103000)
#define CXD56_IMG_SPI_BASE        (CXD56_ADSP_BASE  + 0x02103400) /* IMG_SSP (Display) */
#define CXD56_IMG_WSPI_BASE       (CXD56_ADSP_BASE  + 0x02103c00) /* IMG_SSP (WiFi) */
#define CXD56_USBDEV_BASE         (CXD56_ADSP_BASE  + 0x02200000)
#define CXD56_EMMC_BASE           (CXD56_ADSP_BASE  + 0x02201000)
#define CXD56_SDIO_BASE           (CXD56_ADSP_BASE  + 0x02202000)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_MEMORYMAP_H */
