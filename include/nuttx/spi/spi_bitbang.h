/****************************************************************************
 * include/nuttx/spi/spi_bitbang.h
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_BITBANG_H
#define __INCLUDE_NUTTX_SPI_SPI_BITBANG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_SPI_BITBANG

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* These are the lower-half handlers that perform the level-level, platform-
 * specific bit-bang operations.
 */

struct spi_bitbang_s; /* Forward reference */
struct spi_bitbang_ops_s
{
  /* Platform specific chip select logic */

  void (*select)(FAR struct spi_bitbang_s *priv, uint32_t devid,
                 bool selected);

  /* Platform-specific, SPI frequency function */

  uint32_t (*setfrequency)(FAR struct spi_bitbang_s *priv,
                           uint32_t frequency);

  /* Platform-specific, SPI mode function */

  void (*setmode)(FAR struct spi_bitbang_s *priv, enum spi_mode_e mode);

  /* Platform-specific word exchange function */

  uint16_t (*exchange)(FAR struct spi_bitbang_s *priv, uint16_t dataout);

  /* Platform-specific word exchange function */

  uint8_t (*status)(FAR struct spi_bitbang_s *priv, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
  /* Platform-specific CMD/DATA function */

  int (*cmddata)(FAR struct spi_bitbang_s *priv, uint32_t devid,
                 bool cmd);
#endif
};

/* This is the type of the function that can exchange one bit */

typedef CODE uint8_t (*bitexchange_t)(uint8_t dataout, uint32_t holdtime);

/* This structure provides the state of the SPI bit-bang driver */

struct spi_bitbang_s
{
  struct spi_dev_s dev;                    /* Publicly visible version of SPI driver */
  FAR const struct spi_bitbang_ops_s *low; /* Low-level operations */
  uint32_t         holdtime;               /* SCK hold time to achieve requested frequency */
  bitexchange_t    exchange;               /* The select bit exchange function */
  mutex_t          lock;                   /* Supports mutually exclusive access to SPI */
  FAR void        *priv;                   /* Private data for instance specific */
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
 * Description:
 *   Create an instance of the SPI bit-bang driver.
 *
 * Input Parameters:
 *   low - Low-level, platform specific device operations.
 *   low_priv - Low-level private data, platform specific data.
 *
 * Returned Value:
 *   On success a non-NULL, initialized SPI driver instance is returned.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_create_bitbang(
                         FAR const struct spi_bitbang_ops_s *low,
                         FAR void *low_priv);

/****************************************************************************
 * Name:  spi_destroy_bitbang
 *
 * Description:
 *   Destroy an instance of the SPI bit-bang driver.
 *
 * Input Parameters:
 *   dev - device instance, target driver to destroy.
 *
 ****************************************************************************/

void spi_destroy_bitbang(FAR struct spi_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SPI_BITBANG */
#endif /* __INCLUDE_NUTTX_SPI_SPI_BITBANG_H */
