/****************************************************************************
 * arch/sim/src/sim/posix/sim_spi.h
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

#ifndef __ARCH_SIM_SRC_POSIX_SIM_SPI_H
#define __ARCH_SIM_SRC_POSIX_SIM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "config.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HWFEAT_CS_INACTIVE (1 << 1)  /* Force CS inactive after transfer. */
#define HWFEAT_CS_ACTIVE   (1 << 2)  /* Force CS active after transfer. */
#define HWFEAT_LSBFIRST    (1 << 4)  /* 1 for LSB 1st, default 0 for MSB 1st*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The types below (spi_dev_s, spi_ops_s, spi_mode_e, spi_hwfeatures_t and
 * spi_mediachange_t) are the same as in nuttx/spi/spi.h.
 */

enum spi_mode_e
{
  SPIDEV_MODE0 = 0,     /* CPOL=0 CHPHA=0 */
  SPIDEV_MODE1,         /* CPOL=0 CHPHA=1 */
  SPIDEV_MODE2,         /* CPOL=1 CHPHA=0 */
  SPIDEV_MODE3,         /* CPOL=1 CHPHA=1 */
  SPIDEV_MODETI,        /* CPOL=0 CPHA=1 TI Synchronous Serial Frame Format */
};

#ifdef CONFIG_SPI_HWFEATURES
typedef uint8_t spi_hwfeatures_t;
#endif

typedef void (*spi_mediachange_t)(void *arg);

struct spi_dev_s;
struct spi_ops_s
{
  int      (*lock)(struct spi_dev_s *dev, bool lock);
  void     (*select)(struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
  uint32_t (*setfrequency)(struct spi_dev_s *dev,
                           uint32_t frequency);
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  int      (*setdelay)(struct spi_dev_s *dev, uint32_t a,
                       uint32_t b, uint32_t c);
#endif
  void     (*setmode)(struct spi_dev_s *dev, enum spi_mode_e mode);
  void     (*setbits)(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
  int      (*hwfeatures)(struct spi_dev_s *dev,
                         spi_hwfeatures_t features);
#endif
  uint8_t  (*status)(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
  int      (*cmddata)(struct spi_dev_s *dev, uint32_t devid,
                      bool cmd);
 #endif
  uint32_t (*send)(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
  void     (*exchange)(struct spi_dev_s *dev,
                       const void *txbuffer, void *rxbuffer,
                       size_t nwords);
#else
  void     (*sndblock)(struct spi_dev_s *dev,
                       const void *buffer, size_t nwords);
  void     (*recvblock)(struct spi_dev_s *dev, void *buffer,
                        size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
  int      (*trigger)(struct spi_dev_s *dev);
#endif
  int      (*registercallback)(struct spi_dev_s *dev,
                               spi_mediachange_t callback, void *arg);
};

struct spi_dev_s
{
  const struct spi_ops_s *ops;
};

/* NuttX SPI transaction struct (ref: spi_transfer.h). */

struct spi_trans_s
{
  /* SPI attributes for unique to this transaction */

  bool deselect;            /* De-select after transfer */
#ifdef CONFIG_SPI_CMDDATA
  bool cmd;                 /* true=command; false=data */
#endif
#ifdef CONFIG_SPI_HWFEATURES
  spi_hwfeatures_t hwfeat;  /* H/W features to enable on this transfer */
#endif
  useconds_t delay;         /* Microsecond delay after transfer */

  /* These describe the single data transfer */

  size_t nwords;            /* Number of words in transfer */
  const void *txbuffer;     /* Source buffer for TX transfer */
  void *rxbuffer;           /* Sink buffer for RX transfer */
};

/* NuttX SPI sequence of transactions (ref: spi_transfer.h). */

struct spi_sequence_s
{
  /* Properties that are fixed throughout the transfer */

  uint32_t dev;                /* See enum spi_devtype_e */
  uint8_t mode;                /* See enum spi_mode_e */
  uint8_t nbits;               /* Number of bits */
  uint8_t ntrans;              /* Number of transactions */
  uint32_t frequency;          /* SPI frequency (Hz) */
#ifdef CONFIG_SPI_CS_DELAY_CONTROL
  uint32_t a;                  /* Arguments to setdelay() */
  uint32_t b;
  uint32_t c;
#endif

  /* A pointer to the list of transfers to be be performed. */

  struct spi_trans_s *trans;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_SIM_SRC_POSIX_SIM_SPI_H */
