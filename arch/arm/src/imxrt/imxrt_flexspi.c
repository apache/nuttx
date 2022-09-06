/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexspi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "barriers.h"

#include "imxrt_gpio.h"
#include "imxrt_periphclks.h"
#include "imxrt_flexspi.h"
#include "hardware/imxrt_flexspi.h"

#ifdef CONFIG_IMXRT_FLEXSPI

/*  There is AHBBUSERROREN bit in INTEN register */
#define FLEXSPI_HAS_INTEN_AHBBUSERROREN (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the FlexSPI controller.
 *
 */

struct imxrt_flexspidev_s
{
  struct flexspi_dev_s flexspi; /* Externally visible part of the FlexSPI
                                 * interface */

  struct flexspi_type_s *base; /* FlexSPI controller register base address */

  bool initialized;            /* TRUE: Controller has been initialized */

  mutex_t lock;                /* Assures mutually exclusive access to
                                * FlexSPI */
};

/* FlexSPI methods */

static int imxrt_flexspi_lock(struct flexspi_dev_s *dev, bool lock);
static int imxrt_flexspi_transfer_blocking(struct flexspi_dev_s *dev,
                                      struct flexspi_transfer_s *xfer);
static void imxrt_flexspi_software_reset(struct flexspi_dev_s *dev);
static void imxrt_flexspi_update_lut(struct flexspi_dev_s *dev,
                                     uint32_t index,
                                     const uint32_t *cmd,
                                     uint32_t count);
static void imxrt_flexspi_set_device_config(struct flexspi_dev_s *dev,
                                      struct flexspi_device_config_s *config,
                                      enum flexspi_port_e port);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FlexSPI0 driver operations */

static const struct flexspi_ops_s g_flexspi0ops =
{
  .lock              = imxrt_flexspi_lock,
  .transfer_blocking = imxrt_flexspi_transfer_blocking,
  .software_reset    = imxrt_flexspi_software_reset,
  .update_lut        = imxrt_flexspi_update_lut,
  .set_device_config  = imxrt_flexspi_set_device_config,
};

/* This is the overall state of the FlexSPI0 controller */

static struct imxrt_flexspidev_s g_flexspi0dev =
{
  .flexspi =
  {
    .ops = &g_flexspi0ops,
  },
  .base = (struct flexspi_type_s *) IMXRT_FLEXSPIC_BASE,
  .lock = NXMUTEX_INITIALIZER,
};

#define FREQ_1MHz             (1000000ul)
#define FLEXSPI_DLLCR_DEFAULT (0x100ul)
#define FLEXSPI_LUT_KEY_VAL   (0x5af05af0ul)

enum
{
  FLEXSPI_DELAY_CELL_UNIT_MIN = 75,  /* 75ps */

  FLEXSPI_DELAY_CELL_UNIT_MAX = 225, /* 225ps */
};

enum
{
  FLEXSPI_FLASH_A_SAMPLE_CLOCK_SLAVE_DELAY_LOCKED =
      FLEXSPI_STS2_ASLVLOCK_MASK, /* Flash A sample clock slave delay line locked */

  FLEXSPI_FLASH_A_SAMPLE_CLOCK_REF_DELAY_LOCKED =
      FLEXSPI_STS2_AREFLOCK_MASK, /* Flash A sample clock reference delay line locked */

  FLEXSPI_FLASH_B_SAMPLE_CLOCK_SLAVE_DELAY_LOCKED =
      FLEXSPI_STS2_BSLVLOCK_MASK, /* Flash B sample clock slave delay line locked */

  FLEXSPI_FLASH_B_SAMPLE_CLOCK_REF_DELAY_LOCKED =
      FLEXSPI_STS2_BREFLOCK_MASK, /* Flash B sample clock reference delay line locked */
};

/* FLEXSPI interrupt status flags */

enum flexspi_flags_e
{
  FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT_FLAG = FLEXSPI_INTEN_SEQTIMEOUTEN_MASK, /* Sequence execution timeout */
#if defined(FLEXSPI_HAS_INTEN_AHBBUSERROREN) && FLEXSPI_HAS_INTEN_AHBBUSERROREN
  FLEXSPI_AHB_BUS_ERROR_FLAG = FLEXSPI_INTEN_AHBBUSERROREN_MASK, /* AHB Bus error flag */
#else
  FLEXSPI_AHB_BUS_TIMEOUT_FLAG = FLEXSPI_INTEN_AHBBUSTIMEOUTEN_MASK, /* AHB Bus timeout */
#endif
  FLEXSPI_SCK_STOPPED_BECAUSE_TX_EMPTY_FLAG =
      FLEXSPI_INTEN_SCKSTOPBYWREN_MASK, /* SCK is stopped during command
                                         * sequence because Async TX FIFO empty */

  FLEXSPI_SCK_STOPPED_BECAUSE_RX_FULL_FLAG =
      FLEXSPI_INTEN_SCKSTOPBYRDEN_MASK, /* SCK is stopped during command
                                         * sequence because Async RX FIFO full */

  FLEXSPI_IP_TX_FIFO_WATERMARK_EMPTY_FLAG     = FLEXSPI_INTEN_IPTXWEEN_MASK, /* IP TX FIFO WaterMark empty */

  FLEXSPI_IP_RX_FIFO_WATERMARK_AVAILABLE_FLAG = FLEXSPI_INTEN_IPRXWAEN_MASK, /* IP RX FIFO WaterMark available */

  FLEXSPI_AHB_COMMAND_SEQUENCE_ERROR_FLAG =
      FLEXSPI_INTEN_AHBCMDERREN_MASK,                                  /* AHB triggered Command Sequences Error */

  FLEXSPI_IP_COMMAND_SEQUENCE_ERROR_FLAG = FLEXSPI_INTEN_IPCMDERREN_MASK, /* IP triggered Command Sequences Error */

  FLEXSPI_AHB_COMMAND_GRANT_TIMEOUT_FLAG =
      FLEXSPI_INTEN_AHBCMDGEEN_MASK, /* AHB triggered Command Sequences Grant Timeout */

  FLEXSPI_IP_COMMAND_GRANT_TIMEOUT_FLAG =
      FLEXSPI_INTEN_IPCMDGEEN_MASK, /* IP triggered Command Sequences Grant Timeout */

  FLEXSPI_IP_COMMAND_EXECUTION_DONE_FLAG =
      FLEXSPI_INTEN_IPCMDDONEEN_MASK,  /* IP triggered Command Sequences Execution finished */

  FLEXSPI_ALL_INTERRUPT_FLAGS = 0xfffu, /* All flags */
};

/* Common sets of flags used by the driver */

enum flexspi_flag_constants_e
{
  /*  Errors to check for */

  ERROR_FLAGS = FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT_FLAG |
                FLEXSPI_IP_COMMAND_SEQUENCE_ERROR_FLAG |
                FLEXSPI_IP_COMMAND_GRANT_TIMEOUT_FLAG,
};

#define FLEXSPI_AHB_BUFFER_COUNT (4)

/* FLEXSPI sample clock source selection for Flash Reading */

enum flexspi_read_sample_clock_e
{
  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY =         0x0u,      /* Dummy Read strobe generated by FlexSPI Controller
                                                                    * and loopback internally */

  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD =       0x1u,      /* Dummy Read strobe generated by FlexSPI Controller
                                                                    * and loopback from DQS pad */

  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_SCK_PAD =       0x2u, /* SCK output clock and loopback from SCK pad */

  FLEXSPI_READ_SAMPLE_CLK_EXTERNAL_INPUT_FROM_DQS_PAD = 0x3u, /* Flash provided Read strobe and input from DQS pad */
};

struct flexspi_ahb_buffer_config_s
{
  uint8_t priority;    /* This priority for AHB Master Read which this AHB RX Buffer is assigned */

  uint8_t master_index; /* AHB Master ID the AHB RX Buffer is assigned */

  uint16_t buffer_size; /* AHB buffer size in byte */

  bool enable_prefetch; /* AHB Read Prefetch Enable for current AHB RX Buffer corresponding Master, allows
                         * prefetch disable/enable separately for each master */
};

/*   FLEXSPI configuration structure */

struct flexspi_config_s
{
  enum flexspi_read_sample_clock_e rx_sample_clock; /* Sample Clock source selection for Flash Reading */

  bool enable_sck_free_running;                 /* Enable/disable SCK output free-running */

  bool enable_combination;                    /* Enable/disable combining PORT A and B Data Pins
                                               * (SIOA[3:0] and SIOB[3:0]) to support Flash Octal mode */

  bool enable_doze;                           /* Enable/disable doze mode support */

  bool enable_half_speed_access;                /* Enable/disable divide by 2 of the clock for half
                                                 * speed commands */

  bool enable_sckb_diff_opt;                    /* Enable/disable SCKB pad use as SCKA differential clock
                                                 * output, when enable, Port B flash access is not available */

  bool enable_same_config_for_all;               /* Enable/disable same configuration for all connected devices
                                                  * when enabled, same configuration in FLASHA1CRx is applied to all */

  uint16_t seq_timeout_cycle;                  /* Timeout wait cycle for command sequence execution,
                                                * timeout after ahb_grant_timeout_cyle*1024 serial root clock cycles */

  uint8_t ip_grant_timeout_cycle;               /* Timeout wait cycle for IP command grant, timeout after
                                                 * ip_grant_timeout_cycle*1024 AHB clock cycles */

  uint8_t tx_watermark;                       /* FLEXSPI IP transmit watermark value */

  uint8_t rx_watermark;                       /* FLEXSPI receive watermark value */

  struct
  {
#if !(defined(FLEXSPI_HAS_NO_MCR0_ATDFEN) && FLEXSPI_HAS_NO_MCR0_ATDFEN)
    bool enable_ahb_write_ip_tx_fifo; /* Enable AHB bus write access to IP TX FIFO */
#endif
#if !(defined(FLEXSPI_HAS_NO_MCR0_ARDFEN) && FLEXSPI_HAS_NO_MCR0_ARDFEN)
    bool enable_ahb_write_ip_rx_fifo; /* Enable AHB bus write access to IP RX FIFO */
#endif
    uint8_t ahb_grant_timeout_cycle; /* Timeout wait cycle for AHB command grant,
                                      * timeout after ahb_grant_timeout_cyle*1024 AHB clock cycles */

    uint16_t ahb_bus_timeout_cycle;  /* Timeout wait cycle for AHB read/write access,
                                      * timeout after ahb_bus_timeout_cycle*1024 AHB clock cycles */

    uint8_t resume_wait_cycle;      /* Wait cycle for idle state before suspended command sequence
                                     * resume, timeout after ahb_bus_timeout_cycle AHB clock cycles */

    struct flexspi_ahb_buffer_config_s buffer[FLEXSPI_AHB_BUFFER_COUNT]; /* AHB buffer size */

    bool enable_clear_ahb_buffer_opt; /* Enable/disable automatically clean AHB RX Buffer and TX Buffer
                                       * when FLEXSPI returns STOP mode ACK */

    bool enable_read_address_opt;    /* Enable/disable remove AHB read burst start address alignment limitation.
                                      * when enable, there is no AHB read burst start address alignment limitation */

    bool enable_ahb_prefetch;       /* Enable/disable AHB read prefetch feature, when enabled, FLEXSPI
                                     * will fetch more data than current AHB burst */

    bool enable_ahb_bufferable;     /* Enable/disable AHB bufferable write access support, when enabled,
                                     * FLEXSPI return before waiting for command execution finished */

    bool enable_ahb_cachable;       /* Enable AHB bus cachable read access support */
  } ahb_config;
};

/****************************************************************************
 * Prototypes
 ****************************************************************************/

/* Check and clear IP command execution errors.
 *
 * @param base FLEXSPI base pointer.
 * @param status interrupt status.
 */

static int imxrt_flexspi_check_and_clear_error(struct flexspi_type_s *base,
                                               uint32_t status);

/****************************************************************************
 * Variables
 ****************************************************************************/

/****************************************************************************
 * Code
 ****************************************************************************/

/* Software reset for the FLEXSPI logic.
 *
 * This function sets the software reset flags for both AHB and buffer domain
 * and resets both AHB buffer and also IP FIFOs.
 *
 * @param base FLEXSPI peripheral base address.
 */

static inline void imxrt_flexspi_software_reset_private(
                        struct flexspi_type_s *base)
{
  base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
  while (0u != (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK))
    {
    }
}

/* Returns whether the bus is idle.
 *
 * @param base FLEXSPI peripheral base address.
 * @retval true Bus is idle.
 * @retval false Bus is busy.
 */

static inline bool imxrt_flexspi_get_bus_idle_status(
                        struct flexspi_type_s *base)
{
  return (0u != (base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK)) &&
          (0u != (base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK));
}

static uint32_t imxrt_flexspi_configure_dll(struct flexspi_type_s *base,
                                  struct flexspi_device_config_s *config)
{
  bool is_unified_config = true;
  uint32_t flexspi_dll_value;
  uint32_t dll_value;
  uint32_t temp;

  uint32_t rx_sample_clock = (base->MCR0 & FLEXSPI_MCR0_RXCLKSRC_MASK) >>
                              FLEXSPI_MCR0_RXCLKSRC_SHIFT;
  switch (rx_sample_clock)
    {
      case (uint32_t)FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY:
      case (uint32_t)FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD:
      case (uint32_t)FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_SCK_PAD:
        is_unified_config = true;
        break;
      case (uint32_t)FLEXSPI_READ_SAMPLE_CLK_EXTERNAL_INPUT_FROM_DQS_PAD:
        if (config->is_sck2_enabled)
          {
            is_unified_config = true;
          }
        else
          {
            is_unified_config = false;
          }
        break;
      default:
          assert(false);
          break;
  }

  if (is_unified_config)
    {
      flexspi_dll_value = FLEXSPI_DLLCR_DEFAULT; /* 1 fixed delay cells in DLL delay chain */
    }
  else
    {
      if (config->flexspi_root_clk >= 100u * FREQ_1MHz)
        {
          /* DLLEN = 1, SLVDLYTARGET = 0xF, */

          flexspi_dll_value = FLEXSPI_DLLCR_DLLEN(1) |
                              FLEXSPI_DLLCR_SLVDLYTARGET(0x0f);
        }
      else
        {
          temp     = (uint32_t)config->data_valid_time * 1000u; /* Convert data valid time in ns to ps */

          dll_value = temp / (uint32_t)FLEXSPI_DELAY_CELL_UNIT_MIN;
          if (dll_value * (uint32_t)FLEXSPI_DELAY_CELL_UNIT_MIN < temp)
            {
              dll_value++;
            }
          flexspi_dll_value = FLEXSPI_DLLCR_OVRDEN(1) |
                              FLEXSPI_DLLCR_OVRDVAL(dll_value);
        }
    }
  return flexspi_dll_value;
}

static int imxrt_flexspi_check_and_clear_error(struct flexspi_type_s *base,
                                               uint32_t status)
{
  int result = 0;

  /* Check for error */

  status &= (uint32_t)ERROR_FLAGS;
  if (0u != status)
    {
      /* Select the correct error code */

      if (0u != (status & (uint32_t)FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT_FLAG))
        {
          result = STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT;
        }
      else if (0u != (status &
                      (uint32_t)FLEXSPI_IP_COMMAND_SEQUENCE_ERROR_FLAG))
        {
          result = STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR;
        }
      else if (0u != (status &
                      (uint32_t)FLEXSPI_IP_COMMAND_GRANT_TIMEOUT_FLAG))
        {
          result = STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT;
        }
      else
        {
          assert(false);
        }

      /* Clear the flags */

      base->INTR |= status;

      /* Reset fifos. These flags clear automatically */

      base->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
      base->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
  }

  return result;
}

/* Initializes the FLEXSPI module and internal state.
 *
 * This function enables the clock for FLEXSPI and also configures the
 * FLEXSPI with the input configure parameters. Users should call this
 * function before any FLEXSPI operations.
 *
 * param base FLEXSPI peripheral base address.
 * param config FLEXSPI configure structure.
 */

void imxrt_flexspi_init(struct flexspi_type_s *base,
                        const struct flexspi_config_s *config)
{
  uint32_t config_value = 0;
  uint8_t i            = 0;

  /* Reset peripheral before configuring it */

  base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
  imxrt_flexspi_software_reset_private(base);

  /* Configure MCR0 configuration items */

  config_value = FLEXSPI_MCR0_RXCLKSRC(config->rx_sample_clock) |
                 FLEXSPI_MCR0_DOZEEN(config->enable_doze) |
                 FLEXSPI_MCR0_IPGRANTWAIT(config->ip_grant_timeout_cycle) |
                 FLEXSPI_MCR0_AHBGRANTWAIT(
                   config->ahb_config.ahb_grant_timeout_cycle) |
                 FLEXSPI_MCR0_SCKFREERUNEN(config->enable_sck_free_running) |
                 FLEXSPI_MCR0_HSEN(config->enable_half_speed_access) |
                 FLEXSPI_MCR0_COMBINATIONEN(config->enable_combination) |
#if !(defined(FLEXSPI_HAS_NO_MCR0_ATDFEN) && FLEXSPI_HAS_NO_MCR0_ATDFEN)
                 FLEXSPI_MCR0_ATDFEN(
                   config->ahb_config.enable_ahb_write_ip_tx_fifo) |
#endif
#if !(defined(FLEXSPI_HAS_NO_MCR0_ARDFEN) && FLEXSPI_HAS_NO_MCR0_ARDFEN)
                 FLEXSPI_MCR0_ARDFEN(
                   config->ahb_config.enable_ahb_write_ip_rx_fifo) |
#endif
                 FLEXSPI_MCR0_MDIS_MASK;
  base->MCR0 = config_value;

  /* Configure MCR1 configurations */

  config_value =
      FLEXSPI_MCR1_SEQWAIT(config->seq_timeout_cycle) |
      FLEXSPI_MCR1_AHBBUSWAIT(config->ahb_config.ahb_bus_timeout_cycle);

  base->MCR1 = config_value;

  /* Configure MCR2 configurations */

  config_value = base->MCR2;
  config_value &= ~(FLEXSPI_MCR2_RESUMEWAIT_MASK |
                    FLEXSPI_MCR2_SCKBDIFFOPT_MASK |
                    FLEXSPI_MCR2_SAMEDEVICEEN_MASK |
                    FLEXSPI_MCR2_CLRAHBBUFOPT_MASK);

  config_value |= FLEXSPI_MCR2_RESUMEWAIT(
                    config->ahb_config.resume_wait_cycle) |
                  FLEXSPI_MCR2_SCKBDIFFOPT(
                    config->enable_sckb_diff_opt) |
                  FLEXSPI_MCR2_SAMEDEVICEEN(
                    config->enable_same_config_for_all) |
                  FLEXSPI_MCR2_CLRAHBBUFOPT(
                    config->ahb_config.enable_clear_ahb_buffer_opt);

  base->MCR2 = config_value;

  /* Configure AHB control items */

  config_value = base->AHBCR;
  config_value &= ~(FLEXSPI_AHBCR_READADDROPT_MASK |
                    FLEXSPI_AHBCR_PREFETCHEN_MASK |
                    FLEXSPI_AHBCR_BUFFERABLEEN_MASK |
                    FLEXSPI_AHBCR_CACHABLEEN_MASK);

  config_value |= FLEXSPI_AHBCR_READADDROPT(
                    config->ahb_config.enable_read_address_opt) |
                  FLEXSPI_AHBCR_PREFETCHEN(
                    config->ahb_config.enable_ahb_prefetch) |
                  FLEXSPI_AHBCR_BUFFERABLEEN(
                    config->ahb_config.enable_ahb_bufferable) |
                  FLEXSPI_AHBCR_CACHABLEEN(
                    config->ahb_config.enable_ahb_cachable);

  base->AHBCR = config_value;

  /* Configure AHB rx buffers */

  for (i = 0; i < (uint32_t)FLEXSPI_AHB_BUFFER_COUNT; i++)
    {
      config_value = base->AHBRXBUFCR0[i];

      config_value &= ~(FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK |
                        FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK |
                        FLEXSPI_AHBRXBUFCR0_MSTRID_MASK |
                        FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK);

      config_value |= FLEXSPI_AHBRXBUFCR0_PREFETCHEN(
                        config->ahb_config.buffer[i].enable_prefetch) |
                      FLEXSPI_AHBRXBUFCR0_PRIORITY(
                        config->ahb_config.buffer[i].priority) |
                      FLEXSPI_AHBRXBUFCR0_MSTRID(
                        config->ahb_config.buffer[i].master_index) |
                      FLEXSPI_AHBRXBUFCR0_BUFSZ((uint32_t)
                        config->ahb_config.buffer[i].buffer_size / 8u);

      base->AHBRXBUFCR0[i] = config_value;
    }

  /* Configure IP FIFO watermarks */

  base->IPRXFCR &= ~FLEXSPI_IPRXFCR_RXWMRK_MASK;
  base->IPRXFCR |=
        FLEXSPI_IPRXFCR_RXWMRK((uint32_t)config->rx_watermark / 8u - 1u);
  base->IPTXFCR &= ~FLEXSPI_IPTXFCR_TXWMRK_MASK;
  base->IPTXFCR |=
        FLEXSPI_IPTXFCR_TXWMRK((uint32_t)config->tx_watermark / 8u - 1u);

  /* Reset flash size on all ports */

  for (i = 0; i < (uint32_t)FLEXSPI_PORT_COUNT; i++)
    {
      base->FLSHCR0[i] = 0;
    }
}

/* Gets default settings for FLEXSPI.
 *
 * param config FLEXSPI configuration structure.
 */

void imxrt_flexspi_get_default_config(struct flexspi_config_s *config)
{
  /* Initializes the configure structure to zero */

  memset(config, 0, sizeof(*config));

  config->rx_sample_clock = FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD;
  config->enable_sck_free_running     = false;
  config->enable_combination          = false;
  config->enable_doze                 = true;
  config->enable_half_speed_access    = false;
  config->enable_sckb_diff_opt        = false;
  config->enable_same_config_for_all  = false;
  config->seq_timeout_cycle           = 0xffff;
  config->ip_grant_timeout_cycle      = 0xff;
  config->tx_watermark                = 8;
  config->rx_watermark                = 8;

#if !(defined(FLEXSPI_HAS_NO_MCR0_ATDFEN) && FLEXSPI_HAS_NO_MCR0_ATDFEN)
  config->ahb_config.enable_ahb_write_ip_tx_fifo = false;
#endif

#if !(defined(FLEXSPI_HAS_NO_MCR0_ARDFEN) && FLEXSPI_HAS_NO_MCR0_ARDFEN)
  config->ahb_config.enable_ahb_write_ip_rx_fifo = false;
#endif

  config->ahb_config.ahb_grant_timeout_cycle  = 0xff;
  config->ahb_config.ahb_bus_timeout_cycle    = 0xffff;
  config->ahb_config.resume_wait_cycle        = 0x20;
  memset(config->ahb_config.buffer, 0,
         sizeof(config->ahb_config.buffer));

  /* Use invalid master ID 0xF and buffer size 0 for the first several
   * buffers.
   */

  for (uint8_t i = 0; i < ((uint8_t)FLEXSPI_AHB_BUFFER_COUNT - 2u); i++)
    {
      /* Default enable AHB prefetch */

      config->ahb_config.buffer[i].enable_prefetch = true;

      /* Invalid master index which is no used, so will never hit */

      config->ahb_config.buffer[i].master_index = 0xf;

      /* Default buffer size 0 for buffer0 to
       * buffer(FLEXSPI_AHB_BUFFER_COUNT - 3)
       */

      config->ahb_config.buffer[i].buffer_size = 0;
    }

  for (uint8_t i = ((uint8_t)FLEXSPI_AHB_BUFFER_COUNT - 2);
       i < (uint8_t)FLEXSPI_AHB_BUFFER_COUNT; i++)
    {
      config->ahb_config.buffer[i].enable_prefetch = true; /* Default enable
                                                            * AHB prefetch.
                                                            */

      config->ahb_config.buffer[i].buffer_size     = 256u; /* Default buffer
                                                            * size 256 bytes.
                                                            */
    }

  config->ahb_config.enable_clear_ahb_buffer_opt  = false;
  config->ahb_config.enable_read_address_opt      = false;
  config->ahb_config.enable_ahb_prefetch          = false;
  config->ahb_config.enable_ahb_bufferable        = false;
  config->ahb_config.enable_ahb_cachable          = false;
}

/* Configures the connected device parameter.
 *
 * This function configures the connected device relevant parameters, such
 * as the size, command, and so on. The flash configuration value cannot have
 * a default value. The user needs to configure it according to the connected
 * device.
 *
 * param base   FLEXSPI peripheral base address.
 * param config Device configuration parameters.
 * param port   FLEXSPI Operation port.
 */

void imxrt_flexspi_set_device_config_private(struct flexspi_type_s *base,
                                    struct flexspi_device_config_s *config,
                                    enum flexspi_port_e port)
{
  uint32_t config_value = 0;
  uint32_t status_value = 0;
  uint8_t index        = (uint8_t)port >> 1u; /* PortA with index 0, PortB with index 1 */

  /* Wait for bus idle before change flash configuration */

  while (!imxrt_flexspi_get_bus_idle_status(base))
    {
    }

  /* Configure flash size */

  base->FLSHCR0[port] = config->flash_size;

  /* Configure flash parameters */

  base->FLSHCR1[port] =
        FLEXSPI_FLSHCR1_CSINTERVAL(config->cs_interval) |
        FLEXSPI_FLSHCR1_CSINTERVALUNIT(config->cs_interval_unit) |
        FLEXSPI_FLSHCR1_TCSH(config->cs_hold_time) |
        FLEXSPI_FLSHCR1_TCSS(config->cs_setup_time) |
        FLEXSPI_FLSHCR1_CAS(config->columnspace) |
        FLEXSPI_FLSHCR1_WA(config->enable_word_address);

  /* Configure AHB operation items */

  config_value = base->FLSHCR2[port];

  config_value &= ~(FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK |
                    FLEXSPI_FLSHCR2_AWRWAIT_MASK |
                    FLEXSPI_FLSHCR2_AWRSEQNUM_MASK |
                    FLEXSPI_FLSHCR2_AWRSEQID_MASK |
                    FLEXSPI_FLSHCR2_ARDSEQNUM_MASK |
                    FLEXSPI_FLSHCR2_ARDSEQID_MASK);

  config_value |=
      FLEXSPI_FLSHCR2_AWRWAITUNIT(config->ahb_write_wait_unit) |
      FLEXSPI_FLSHCR2_AWRWAIT(config->ahb_write_wait_interval);

  if (config->awr_seq_number > 0u)
    {
      config_value |= FLEXSPI_FLSHCR2_AWRSEQID(
                       (uint32_t)config->awr_seq_index) |
                      FLEXSPI_FLSHCR2_AWRSEQNUM(
                       (uint32_t)config->awr_seq_number - 1u);
    }

  if (config->ard_seq_number > 0u)
    {
      config_value |= FLEXSPI_FLSHCR2_ARDSEQID(
                       (uint32_t)config->ard_seq_index) |
                      FLEXSPI_FLSHCR2_ARDSEQNUM(
                       (uint32_t)config->ard_seq_number - 1u);
    }

  base->FLSHCR2[port] = config_value;

  /* Configure DLL */

  config_value        = imxrt_flexspi_configure_dll(base, config);
  base->DLLCR[index] = config_value;

  /* Configure write mask */

  if (config->enable_write_mask)
    {
      base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMOPT1_MASK;
    }
  else
    {
      base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMOPT1_MASK;
    }

  if (index == 0u) /* Port A */
    {
      base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMENA_MASK;
      base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMENA(config->enable_write_mask);
    }
  else
    {
      base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMENB_MASK;
      base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMENB(config->enable_write_mask);
    }

  /* Exit stop mode */

  base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

  /* According to ERR011377, need to delay at least 100 NOPs to ensure the
   * DLL is locked.
   */

  status_value =
      (index == 0u) ?
          ((uint32_t)FLEXSPI_FLASH_A_SAMPLE_CLOCK_SLAVE_DELAY_LOCKED |
           (uint32_t)FLEXSPI_FLASH_A_SAMPLE_CLOCK_REF_DELAY_LOCKED) :
          ((uint32_t)FLEXSPI_FLASH_B_SAMPLE_CLOCK_SLAVE_DELAY_LOCKED |
           (uint32_t)FLEXSPI_FLASH_B_SAMPLE_CLOCK_REF_DELAY_LOCKED);

  if (0u != (config_value & FLEXSPI_DLLCR_DLLEN_MASK))
    {
      /* Wait slave delay line locked and slave reference delay line locked */

      while ((base->STS2 & status_value) != status_value)
        {
        }

      /* Wait at least 100 NOPs */

      for (uint8_t delay = 100u; delay > 0u; delay--)
        {
          asm("NOP");
        }
    }
}

/* Updates the LUT table.
 *
 * param base  FLEXSPI peripheral base address.
 * param index From which index start to update.
 *             It could be any index of the LUT table, which also allows
 *             user to update command content inside a command. Each command
 *             consists of up to 8 instructions and occupy 4*32-bit memory.
 * param cmd   Command sequence array.
 * param count Number of sequences.
 */

void imxrt_flexspi_update_lut_private(struct flexspi_type_s *base,
                                      uint32_t index,
                                      const uint32_t *cmd,
                                      uint32_t count)
{
  assert(index < 64u);

  uint32_t i = 0;
  volatile uint32_t *lut_base;

  /* Wait for bus idle before change flash configuration */

  while (!imxrt_flexspi_get_bus_idle_status(base))
    {
    }

  /* Unlock LUT for update */

  base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
  base->LUTCR  = 0x02;

  lut_base = &base->LUT[index];
  for (i = 0; i < count; i++)
    {
      *lut_base++ = *cmd++;
    }

  /* Lock LUT */

  base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
  base->LUTCR  = 0x01;
}

/* Sends a buffer of data bytes using blocking method.
 * note This function blocks via polling until all bytes have been sent.
 * param base FLEXSPI peripheral base address
 * param buffer The data bytes to send
 * param size The number of data bytes to send
 * retval 0 write success without error
 * STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT sequence execution timeout
 * STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR IP command sequence error
 * detected
 * STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT IP command grant
 * timeout detected.
 */

static int imxrt_flexspi_write_blocking(struct flexspi_type_s *base,
                                        uint32_t *buffer, size_t size)
{
  uint32_t tx_watermark = ((base->IPTXFCR & FLEXSPI_IPTXFCR_TXWMRK_MASK) >>
                           FLEXSPI_IPTXFCR_TXWMRK_SHIFT) + 1u;
  uint32_t status;
  int result = 0;
  uint32_t i      = 0;

  /* Send data buffer */

  while (0u != size)
    {
      /* Wait until there is room in the fifo. This also checks for errors */

      while (0u == ((status = base->INTR) &
             (uint32_t)FLEXSPI_IP_TX_FIFO_WATERMARK_EMPTY_FLAG))
        {
        }

      result = imxrt_flexspi_check_and_clear_error(base, status);

      if (0 != result)
        {
          return result;
        }

      /* Write watermark level data into tx fifo  */

      if (size >= 8u * tx_watermark)
        {
          for (i = 0u; i < 2u * tx_watermark; i++)
            {
              base->TFDR[i] = *buffer++;
            }

          size = size - 8u * tx_watermark;
        }
      else
        {
          for (i = 0u; i < (size / 4u + 1u); i++)
            {
              base->TFDR[i] = *buffer++;
            }
          size = 0u;
        }

      /* Push a watermark level data into IP TX FIFO */

      base->INTR |= (uint32_t)FLEXSPI_IP_TX_FIFO_WATERMARK_EMPTY_FLAG;
    }

  return result;
}

/* Receives a buffer of data bytes using a blocking method.
 * note This function blocks via polling until all bytes have been sent.
 * param base FLEXSPI peripheral base address
 * param buffer The data bytes to send
 * param size The number of data bytes to receive
 * retval 0 read success without error
 * retval STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT sequence execution
 * timeout retval STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR IP command
 * sequence error detected retval STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT
 * IP command grant timeout detected.
 */

static int imxrt_flexspi_read_blocking(struct flexspi_type_s *base,
                                       uint32_t *buffer, size_t size)
{
  uint32_t rx_watermark = ((base->IPRXFCR & FLEXSPI_IPRXFCR_RXWMRK_MASK) >>
                          FLEXSPI_IPRXFCR_RXWMRK_SHIFT) + 1u;
  uint32_t status;
  int result = 0;
  uint32_t i      = 0;
  bool is_return   = false;

  /* Send data buffer */

  while (0u != size)
    {
      if (size >= 8u * rx_watermark)
        {
          /* Wait until there is room in the fifo. This also checks for
           * errors.
           */

          while (0u == ((status = base->INTR) &
                 (uint32_t)FLEXSPI_IP_RX_FIFO_WATERMARK_AVAILABLE_FLAG))
            {
              result = imxrt_flexspi_check_and_clear_error(base, status);

              if (0 != result)
                {
                  is_return = true;
                  break;
                }
            }
        }
      else
        {
          /* Wait fill level. This also checks for errors */

          while (size > ((((base->IPRXFSTS) & FLEXSPI_IPRXFSTS_FILL_MASK) >>
                 FLEXSPI_IPRXFSTS_FILL_SHIFT) * 8u))
            {
              result = imxrt_flexspi_check_and_clear_error(base, base->INTR);

              if (0 != result)
                {
                  is_return = true;
                  break;
                }
            }
        }

      if (is_return)
        {
          break;
        }

      result = imxrt_flexspi_check_and_clear_error(base, base->INTR);

      if (0 != result)
        {
          break;
        }

      /* Read watermark level data from rx fifo  */

      if (size >= 8u * rx_watermark)
        {
          for (i = 0u; i < 2u * rx_watermark; i++)
            {
              *buffer++ = base->RFDR[i];
            }

          size = size - 8u * rx_watermark;
        }
      else
        {
          for (i = 0u; i < ((size + 3u) / 4u); i++)
            {
              *buffer++ = base->RFDR[i];
            }
          size = 0;
        }

      /* Pop out a watermark level datas from IP RX FIFO */

      base->INTR |= (uint32_t)FLEXSPI_IP_RX_FIFO_WATERMARK_AVAILABLE_FLAG;
    }

  return result;
}

/* Brief Execute command to transfer a buffer data bytes using a blocking
 * method. param base FLEXSPI peripheral base address param xfer pointer to
 * the transfer structure. retval 0 command transfer success without error
 * retval STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT sequence execution
 * timeout retval STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR IP command
 * sequence error detected retval STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT
 * IP command grant timeout detected.
 */

int imxrt_flexspi_transfer_blocking_private(struct flexspi_type_s *base,
                                            struct flexspi_transfer_s *xfer)
{
  uint32_t config_value = 0;
  int result      = 0;

  /* Clear sequence pointer before sending data to external devices */

  base->FLSHCR2[xfer->port] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;

  /* Clear former pending status before start this transfer */

  base->INTR |= FLEXSPI_INTR_AHBCMDERR_MASK |
                FLEXSPI_INTR_IPCMDERR_MASK |
                FLEXSPI_INTR_AHBCMDGE_MASK |
                FLEXSPI_INTR_IPCMDGE_MASK;

  /* Configure base address */

  base->IPCR0 = xfer->device_address;

  /* Reset fifos */

  base->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
  base->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;

  /* Configure data size */

  if ((xfer->cmd_type == FLEXSPI_READ)  ||
      (xfer->cmd_type == FLEXSPI_WRITE) ||
      (xfer->cmd_type == FLEXSPI_CONFIG))
    {
      config_value = FLEXSPI_IPCR1_IDATSZ(xfer->data_size);
    }

  /* Configure sequence ID */

  config_value |=
      FLEXSPI_IPCR1_ISEQID((uint32_t)xfer->seq_index) | \
      FLEXSPI_IPCR1_ISEQNUM((uint32_t)xfer->seq_number - 1u);
  base->IPCR1 = config_value;

  /* Start Transfer */

  base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;

  if ((xfer->cmd_type == FLEXSPI_WRITE) ||
      (xfer->cmd_type == FLEXSPI_CONFIG))
    {
      result = imxrt_flexspi_write_blocking(base, xfer->data,
                                            xfer->data_size);
    }
  else if (xfer->cmd_type == FLEXSPI_READ)
    {
      result = imxrt_flexspi_read_blocking(base, xfer->data,
                                           xfer->data_size);
    }
  else
    {
      /* Empty else */
    }

  /* Wait for bus idle */

  while (!imxrt_flexspi_get_bus_idle_status(base))
    {
    }

  if (xfer->cmd_type == FLEXSPI_COMMAND)
    {
      result = imxrt_flexspi_check_and_clear_error(base, base->INTR);
    }

  return result;
}

/****************************************************************************
 * Name: imxrt_flexspi_lock
 *
 * Description:
 *   On FlexSPI buses where there are multiple devices, it will be necessary
 *   to lock FlexSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock FlexSPI bus, false: unlock FlexSPI bus
 *
 * Returned Value:
 *   Semaphore status
 *
 ****************************************************************************/

static int imxrt_flexspi_lock(struct flexspi_dev_s *dev, bool lock)
{
  struct imxrt_flexspidev_s *priv = (struct imxrt_flexspidev_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_flexspi_transfer_blocking
 *
 * Description:
 *   Perform one FlexSPI transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   xfer    - Describes the transfer to be performed.
 *
 * Returned Value:
 *   0 on SUCCESS, STATUS_FLEXSPI_SEQUENCE_EXECUTION_TIMEOUT,
 *   STATUS_FLEXSPI_IP_COMMAND_SEQUENCE_ERROR or
 *   STATUS_FLEXSPI_IP_COMMAND_GRANT_TIMEOUT otherwise
 *
 ****************************************************************************/

static int imxrt_flexspi_transfer_blocking(struct flexspi_dev_s *dev,
                                           struct flexspi_transfer_s *xfer)
{
  struct imxrt_flexspidev_s *priv = (struct imxrt_flexspidev_s *)dev;

  return (int)imxrt_flexspi_transfer_blocking_private(priv->base, xfer);
}

/****************************************************************************
 * Name: imxrt_flexspi_software_reset
 *
 * Description:
 *   Performs a software reset of FlexSPI
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_flexspi_software_reset(struct flexspi_dev_s *dev)
{
  struct imxrt_flexspidev_s *priv = (struct imxrt_flexspidev_s *)dev;

  imxrt_flexspi_software_reset_private(priv->base);
}

/****************************************************************************
 * Name: imxrt_flexspi_update_lut
 *
 * Description:
 *   Perform FlexSPI LUT table update
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   index   - Index start to update
 *   cmd     - Command array
 *   count   - Size of the array
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void imxrt_flexspi_update_lut(struct flexspi_dev_s *dev,
                                     uint32_t index,
                                     const uint32_t *cmd,
                                     uint32_t count)
{
  struct imxrt_flexspidev_s *priv = (struct imxrt_flexspidev_s *)dev;

  imxrt_flexspi_update_lut_private(priv->base, index, cmd, count);
}

/****************************************************************************
 * Name: imxrt_flexspi_set_device_config
 *
 * Description:
 *   Perform FlexSPI device config
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   config  - Config data for external device
 *   port    - Port
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void imxrt_flexspi_set_device_config(struct flexspi_dev_s *dev,
                                    struct flexspi_device_config_s *config,
                                    enum flexspi_port_e port)
{
  struct imxrt_flexspidev_s *priv = (struct imxrt_flexspidev_s *)dev;

  imxrt_flexspi_set_device_config_private(priv->base, config, port);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_flexspi_initialize
 *
 * Description:
 *   Initialize the selected FlexSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid FlexSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct flexspi_dev_s *imxrt_flexspi_initialize(int intf)
{
  struct imxrt_flexspidev_s *priv;
  struct flexspi_config_s flexspi_config;

  /* The supported i.MXRT parts have only a single FlexSPI port. Other
   * is reserved for code XIP
   */

  DEBUGASSERT(intf >= 0 && intf < 1);

  /* Select the FlexSPI interface */

  if (intf == 0)
    {
      /* If this function is called multiple times, the following operations
       * will be performed multiple times.
       */

      /* Select FlexSPI */

      priv = &g_flexspi0dev;

      /* Enable clocking to the FlexSPI peripheral */

      imxrt_clockrun_flexspi();
    }
  else
    {
      return NULL;
    }

  /* Has the FlexSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* No perform one time initialization */

      /* Perform hardware initialization. Puts the FlexSPI into an active
       * state.
       */

      imxrt_flexspi_get_default_config(&flexspi_config);
      imxrt_flexspi_init(priv->base, &flexspi_config);

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
    }

  return &priv->flexspi;
}

#endif /* CONFIG_IMXRT_FLEXSPI */
