/************************************************************************************
 * arch/arm/src/lpc17xx/chip.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_LPC17XX_LPC1769) || defined(CONFIG_LPC17XX_LPC1768)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1767)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         0  /* No USB device controller */
#  define LPC17_NCAN            0  /* No CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1766)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1765)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1764)
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#elif defined(CONFIG_LPC17XX_LPC1759)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1758)
#  define LPC17_FLASH_SIZE      (512*1024) /* 512Kb */
#  define LPC17_SRAM_SIZE       (64*1024)  /*  64Kb */
#  define LPC17_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1756)
#  define LPC17_FLASH_SIZE      (256*1024) /* 256Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            2  /* Two CAN controllers */
#  define LPC17_NI2S            1  /* One I2S module */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1754)
#  define LPC17_FLASH_SIZE      (128*1024) /* 128Kb */
#  define LPC17_SRAM_SIZE       (32*1024)  /*  32Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        1  /* One USB host controller */
#  define LPC17_NUSBOTG         1  /* One USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            1  /* One DAC module */
#elif defined(CONFIG_LPC17XX_LPC1752)
#  define LPC17_FLASH_SIZE      (64*1024) /* 65Kb */
#  define LPC17_SRAM_SIZE       (16*1024) /* 16Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#elif defined(CONFIG_LPC17XX_LPC1751)
#  define LPC17_FLASH_SIZE      (32*1024) /* 32Kb */
#  define LPC17_SRAM_SIZE       (8*1024)  /*  8Kb */
#  define LPC17_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC17_NUSBHOST        0  /* No USB host controller */
#  define LPC17_NUSBOTG         0  /* No USB OTG controller */
#  define LPC17_NUSBDEV         1  /* One USB device controller */
#  define LPC17_NCAN            1  /* One CAN controller */
#  define LPC17_NI2S            0  /* No I2S modules */
#  define LPC17_NDAC            0  /* No DAC module */
#else
#  error "Unsupported STM32 chip"
#endif

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "lpc17xx_memorymap.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_H */
