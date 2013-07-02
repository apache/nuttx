/****************************************************************************
 * include/nuttx/spi/spi_bitbang.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_BITBANG_H
#define __INCLUDE_NUTTX_SPI_SPI_BITBANG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>

#include <nuttx/spi/spi.h>

#ifdef CONFIG_SPI_BITBANG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Check if SPI debut is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
#ifndef __ASSEMBLY__

/* These are the lower-half handlers that perform the level-level, platform-
 * specific bit-bang operations.
 */

struct spi_bitbang_s; /* Forward reference */
struct spi_bitbang_ops_s
{
  /* Platform specific chip select logic */

  void (*select)(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                 bool selected);

  /* Platform-specific, SPI frequency function */

  uint32_t (*setfrequency)(FAR struct spi_bitbang_s *priv,
                           uint32_t frequency);

  /* Platform-specific, SPI mode function */

  void (*setmode)(FAR struct spi_bitbang_s *priv, enum spi_mode_e mode);

  /* Platform-specific word exchange function */

  uint16_t (*exchange)(FAR struct spi_bitbang_s *priv, uint16_t dataout);

  /* Platform-specific word exchange function */

  uint8_t (*status)(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid);

#ifdef CONFIG_SPI_CMDDATA
  /* Platform-specific CMD/DATA function */

  int (*cmddata)(FAR struct spi_bitbang_s *priv, enum spi_dev_e devid,
                 bool cmd);
#endif
};

/* This is the type of the function that can exchange one bit */

typedef uint8_t (*bitexchange_t)(uint8_t dataout, uint32_t holdtime);

/* This structure provides the state of the SPI bit-bang driver */

struct spi_bitbang_s
{
  struct spi_dev_s dev;                    /* Publicly visible version of SPI driver */
  FAR const struct spi_bitbang_ops_s *low; /* Low-level operations */
  uint32_t         holdtime;               /* SCK hold time to achieve requested frequency */
  bitexchange_t    exchange;               /* The select bit exchange function */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;                /* Supports mutually exclusive access to SPI */
#endif
#ifdef CONFIG_SPI_BITBANG_VARWIDTH
  uint8_t          nbits;                  /* Number of bits in the transfer */
#endif
};

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

/****************************************************************************
 * Name:  spi_create_bitbang
 *
 * Descripton:
 *   Create an instance of the SPI bit-bang driver.
 *
 * Input Parameters:
 *   low - Low-level, platform specific device operations.
 *
 * Returned Value:
 *   On success a non-NULL, initialized SPI driver instance is returned.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_create_bitbang(FAR const struct spi_bitbang_ops_s *low);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SPI_BITBANG */
#endif /* __INCLUDE_NUTTX_SPI_SPI_BITBANG_H */
