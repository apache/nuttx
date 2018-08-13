/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_emc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "lpc54_config.h"

#ifdef CONFIG_LPC54_EMC

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* EMC Feedback clock input source selection */

enum emc_fbclksrc_e
{
  EMC_INTLOOPBACK = 0, /* Use the internal loop back from EMC_CLK output */
  EMC_FBCLLK           /* Use the external EMC_FBCLK input */
};

/* EMC module basic configuration structure */

struct emc_config_s
{
  bool bigendian;      /* True: Memory is big-endian */
  uint8_t clksrc;      /* The feedback clock source */
  uint8_t clkdiv;      /* EMC_CLK = AHB_CLK / (emc_clkDiv + 1). */
};

/* EMC dynamic read strategy */

enum emc_dynamic_read_e
{
  EMC_NODELAY = 0,     /* No delay */
  EMC_CMDDELAY,        /* Command delayed strategy, using EMCCLKDELAY */
  EMC_CMDDELAYPLUS1,   /* Command delayed strategy pluse one clock cycle
                        * using EMCCLKDELAY */
  EMC_CMDDELAYPLUS2,   /* Command delayed strategy pulse two clock cycle
                        * using EMCCLKDELAY */
};

/* EMC dynamic timing/delay configure structure */

struct emc_dynamic_timing_config_s
{
  uint8_t rdconfig;    /* Dynamic read strategy (see enum emc_dynamic_read_e) */
  uint32_t refresh;    /* The refresh period in units of nanoseconds */
  uint32_t rp;         /* Precharge command period in units of nanoseconds */
  uint32_t ras;        /* Active to precharge command period in units of
                        * nanoseconds */
  uint32_t srex;       /* Self-refresh exit time in units of nanoseconds */
  uint32_t apr;        /* Last data out to active command time in units of
                        * nanoseconds */
  uint32_t dal;        /* Data-in to active command in units of nanoseconds */
  uint32_t wr;         /* Write recovery time in unit of nanosecond */
  uint32_t rc;         /* Active to active command period in units of
                        * nanoseconds */
  uint32_t rfc;        /* Auto-refresh period and auto-refresh to active
                        * command period in unit of nanosecond */
  uint32_t xsr;        /* Exit self-refresh to active command time in units
                        * of nanoseconds */
  uint32_t rrd;        /* Active bank A to active bank B latency in units of
                        * nanoseconds */
  uint8_t mrd;         /* Load mode register to active command time in units
                        * of EMCCLK cycles */
};

/* EMC dynamic memory device */

enum emc_dynamic_device_e
{
  EMC_SDRAM = 0,       /* Dynamic memory device: SDRAM */
  EMC_LPSDRAM          /* Dynamic memory device: Low-power SDRAM */
};

/* EMC dynamic memory controller independent chip configuration structure */

struct emc_dynamic_chip_config_s
{
  uint8_t chndx;       /* Chip Index, range from 0 ~ EMC_DYNAMIC_MEMDEV_NUM - 1 */
  uint8_t dyndev;      /* All chips shall use the same device setting. mixed
                        * use are not supported */
  uint8_t rasnclk;     /* Active to read/write delay tRCD */
  uint16_t mode;       /* Sdram mode register setting */
  uint16_t extmode;    /* Used for low-power sdram device. The extended mode
                        * register */
  uint8_t addrmap;     /* Dynamic device address mapping, choose the address
                        * mapping for your specific device */
};

/* EMC memory width for static memory device */

enum emc_static_memwidth_e
{
  EMC_WIDTH_8BIT = 0,  /* 8 bit memory width */
  EMC_WIDTH_16BIT,     /* 16 bit memory width */
  EMC_WIDTH_32BIT      /* 32 bit memory width */
};

/* Define EMC static "special" configuration */

enum emc_static_special_config_e
{
  EMC_ASYNCPAGEENABLE = 0x00000008,  /* Enable the asynchronous page mode.
                                      * page length four */
  EMC_ACTIVEHIGHCS    = 0x00000040,  /* Chip select active high */
  EMC_BYTELANELOW     = 0x00000080,  /* Reads/writes the respective value
                                      * bits in BLS3:0 are low */
  EMC_EXTWAITENABLE   = 0x00000100,  /* Extended wait enable */
  EMC_BUFFERENABLE    = 0x00080000   /* Buffer enable */
};

/* EMC static memory controller independent chip configuration structure */

struct emc_static_chip_config_s
{
  uint8_t chndx;
  uint8_t memwidth;      /* Memory width */
  uint32_t specconfig;   /* Static configuration, OR of enum
                          * emc_static_special_config_e settings */
  uint32_t waitwriteen;  /* The delay form chip select to write enable in
                          * units of nanoseconds */
  uint32_t waitouten;    /* The delay from chip selcet to output enable in
                          * units of nanoseconds */
  uint32_t waitread;     /* In No-page mode, the delay from chip select to
                          * read access in units of nanoseconds */
  uint32_t waitreadpage; /* In page mode, the read after the first read
                          * wait states in units of nanoseconds */
  uint32_t waitwrite;    /* The delay from chip select to write access in
                          * units of nanoseconds */
  uint32_t waitturn;     /* The Bus turn-around time in units of
                          * nanoseconds */
};

/***************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_emc_initialize
 *
 * Description:
 *   This function enables the EMC clock, initializes the emc system
 *   configuration, and enable the EMC module.
 *
 * Input Parameters:
 *   config - Describes the EMC configuration.
 *
 ****************************************************************************/

void lpc54_emc_initialize(FAR const struct emc_config_s *config);

/****************************************************************************
 * Name: lpc54_emc_sdram_initialize
 *
 * Description:
 *   This function initializes the dynamic memory controller in external
 *   memory controller. This function must be called after lpc54_emc_initialize
 *   and before accessing the external dynamic memory.
 *
 * Input Parameters:
 *   timing   - The timing and latency for dynamica memory controller
 *              setting. It will be used for all dynamic memory chips,
 *              therefore the worst timing value for all used chips must be
 *              given.
 *   chconfig - The EMC dynamic memory controller chip-independent
 *              configuration array.  The dimension of the array is given by
 *              nchips.
 *   nchips   - The number of chips to configure and the dimension of the
 *              chconfig array.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC_DYNAMIC
void lpc54_emc_sdram_initialize(FAR const struct emc_dynamic_timing_config_s *timing,
                                FAR const struct emc_dynamic_chip_config_s *chconfig,
                                unsigned int nchips);
#endif /* CONFIG_LPC54_EMC_DYNAMIC */

/****************************************************************************
 * Name: lpc54_emc_sram_initialize
 *
 * Description:
 *   This function initializes the static memory controller in external
 *   memory controller. This function must be called after lpc54_emc_initialize
 *   and before accessing the external dynamic memory.
 *
 * Input Parameters:
 *   extwait    - The extended wait timeout or the read/write transfer time.
 *                This is common for all static memory chips and set with
 *                NULL if not required.
 *   statconfig - The EMC static memory controller chip independent
 *                configuration array.  The dimension of the array is nchips.
 *   nchips     - The total static memory chip numbers been used and the
 *                number of entries in the statconfig array.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_EMC_STATIC
void lpc54_emc_sram_initialize(FAR uint32_t *extwait,
                               FAR const struct emc_static_chip_config_s *statconfig,
                               uint32_t nchips);
#endif /* CONFIG_LPC54_EMC_STATIC */

#endif /* CONFIG_LPC54_EMC */
#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H */
