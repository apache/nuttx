/****************************************************************************
 * arch/avr/src/at32uc3/chip.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_CHIP_H
#define __ARCH_AVR_SRC_AT32UC3_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get customizations for each supported chip */

/* UC3 A0/A1 Series */

/* UC3 A2/A3 Series */

/* UC3 B0 (64-pin) / B1 (48-pin, no USB host) Series */

#ifdef CONFIG_ARCH_CHIP_AT32UC3B064
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (64*1024)    /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (16*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B0128)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (128*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B0256)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (256*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B0512)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (512*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (96*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B164)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (64*1024)    /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (16*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B1128)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (128*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B1256)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (256*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B1512)
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (512*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (96*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#else
#  error "Unsupported AVR32 chip"
#endif

/* Include only the memory map.  Other chip hardware files should then
 * include this file for the proper setup
 */

#include "at32uc3_memorymap.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_CHIP_H */
