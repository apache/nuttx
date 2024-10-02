/****************************************************************************
 * include/nuttx/can/sja1000.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_CAN_SJA1000_H
#define __INCLUDE_NUTTX_CAN_SJA1000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* CAN hardware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */

struct can_bittiming_const_s
{
  char name[16];      /* Name of the CAN controller hardware */
  uint32_t tseg1_min; /* Time segment 1 = prop_seg + phase_seg1 */
  uint32_t tseg1_max;
  uint32_t tseg2_min; /* Time segment 2 = phase_seg2 */
  uint32_t tseg2_max;
  uint32_t sjw_max; /* Synchronisation jump width */
  uint32_t brp_min; /* Bit-rate prescaler */
  uint32_t brp_max;
  uint32_t brp_inc;
};

/* Type of the SJA1000 interrupt handling callback */

struct sja1000_config_s; /* Forward reference */
typedef CODE void (*sja1000_handler_t)(FAR struct sja1000_config_s *config,
                                       FAR void *arg);

/* A reference to a structure of this type must be passed to the SJA1000
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the SJA1000 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct sja1000_config_s
{
  /* Device configuration */

  const struct can_bittiming_const_s *bittiming_const;
  uint8_t port;       /* SJA1000 device port number */
  uint8_t periph;     /* Peripheral ID (optional) */
  uint8_t irq;        /* IRQ associated with this SJA1000 */
  uint8_t cpu;        /* CPU ID */
  int8_t cpuint;      /* CPU interrupt assigned to this SJA1000 */
  uint32_t bitrate;   /* Configured bit rate */
  uint32_t samplep;   /* Configured sample point */
  uint32_t sjw;       /* Synchronization jump width */
  uint32_t clk_freq;  /* Peripheral clock frequency */
  bool loopback;      /* Enable loopback mode */
  bool triple_sample; /* Enable triple-sampling of CAN BUS */

  /* Device characterization */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach interrupt handler to the interrupt
   * detach  - Detach interrupt handler from the interrupt
   */

  CODE int (*attach)(FAR struct sja1000_config_s *config,
                     sja1000_handler_t handler, FAR void *arg);
  CODE int (*detach)(FAR struct sja1000_config_s *config);
};

/* This structure provides the current state of a CAN peripheral */

struct sja1000_dev_s
{
  FAR struct sja1000_config_s *config;  /* The constant configuration */
  uint8_t filters;                      /* STD/EXT filter bit allocator. */
  uint8_t nalloc;                       /* Number of allocated filters */
  uint32_t base;                        /* SJA1000 register base address */

#ifdef CONFIG_ARCH_HAVE_MULTICPU
  spinlock_t lock; /* Device specific lock */
#endif             /* CONFIG_ARCH_HAVE_MULTICPU */

  /* Register read/write callbacks.  These operations all hidden behind
   * callbacks to isolate the driver from differences in register read/write
   * handling by varying boards and MCUs.
   *
   * getreg  - Read from a register address
   * putreg  - Write to a register address
   */

  CODE uint32_t (*getreg)(struct sja1000_dev_s *sja_priv, uint32_t reg);
  CODE void (*putreg)(struct sja1000_dev_s *sja_priv, uint32_t reg,
                      uint32_t value);
  CODE void (*modifyreg32)(struct sja1000_dev_s *sja_priv, uint32_t reg,
                           uint32_t clearbits, uint32_t setbits);
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifdef CONFIG_CAN_SJA1000

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: sja1000_getreg
 *
 * Description:
 *   Read the value of an SJA1000 register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The SJA1000 register address to read
 *
 * Returned Value:
 *
 ****************************************************************************/

static inline uint32_t sja1000_getreg(struct sja1000_dev_s *sja_priv,
    uint32_t reg)
{
  return sja_priv->getreg(sja_priv, reg);
}

/****************************************************************************
 * Name: sja1000_putreg
 *
 * Description:
 *   Set the value of an SJA1000 register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The SJA1000 register address to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sja1000_putreg(struct sja1000_dev_s *sja_priv,
    uint32_t reg, uint32_t value)
{
  sja_priv->putreg(sja_priv, reg, value);
}

/****************************************************************************
 * Name: sja1000_modifyreg32
 *
 * Description:
 *   Modify the value of an SJA1000 register.
 *
 * Input Parameters:
 *   priv - sja1000 lower-half driver context
 *   reg - The SJA1000 register address to modify
 *   clearbits - Bitmask of the bits to clear in a register
 *   setbits - Bitmask of the bits to set in a register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sja1000_modifyreg32(struct sja1000_dev_s *sja_priv,
    uint32_t reg, uint32_t clearbits, uint32_t setbits)
{
  sja_priv->modifyreg32(sja_priv, reg, clearbits, setbits);
}

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sja1000_instantiate
 *
 * Description:
 *   Initialize the selected SJA1000 CAN Bus Controller
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" CAN driver state structure.
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;

FAR struct can_dev_s *sja1000_instantiate(FAR struct sja1000_dev_s *priv);

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_CAN_SJA1000 */

#endif /* __INCLUDE_NUTTX_CAN_SJA1000_H */
