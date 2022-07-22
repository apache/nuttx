/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_fs26.c
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

/* Copyright 2022 NXP
 * This FS26 driver is intended for ENGINEERING DEVELOPMENT OR EVALUATION
 * PURPOSES ONLY.  It is provided as an example to disable the FS26 watchdog
 * functionality for development on the S32K3XX platform.  Please refer to
 * the datasheets and application hints provided on NXP.com to implement
 * full functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"

#include "chip.h"

#include "s32k3xx_pin.h"
#include "hardware/s32k3xx_pinmux.h"
#include "hardware/s32k3xx_lpspi.h"
#include "hardware/s32k3xx_fs26.h"
#include "s32k3xx_fs26.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_lpspi.h"

#include <arch/board/board.h>

#define SWAP_ENDIANESS

#if defined(CONFIG_S32K3XX_FS26)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRC polynomial used for SPI communication. */

#define FS26_CRC_TBL_SIZE 256U     /* Size of CRC table. */
#define FS26_COM_CRC_POLYNOM 0x1DU /* CRC polynom. */
#define FS26_COM_CRC_INIT 0xFFU    /* CRC initial value. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fs26_dev_s g_fs26;

/* CRC lookup table. */

static const uint8_t FS26_CRC_TABLE[FS26_CRC_TBL_SIZE] =
{
  0x00u, 0x1du, 0x3au, 0x27u, 0x74u, 0x69u, 0x4eu, 0x53u, 0xe8u,
  0xf5u, 0xd2u, 0xcfu, 0x9cu, 0x81u, 0xa6u, 0xbbu, 0xcdu, 0xd0u,
  0xf7u, 0xeau, 0xb9u, 0xa4u, 0x83u, 0x9eu, 0x25u, 0x38u, 0x1fu,
  0x02u, 0x51u, 0x4cu, 0x6bu, 0x76u, 0x87u, 0x9au, 0xbdu, 0xa0u,
  0xf3u, 0xeeu, 0xc9u, 0xd4u, 0x6fu, 0x72u, 0x55u, 0x48u, 0x1bu,
  0x06u, 0x21u, 0x3cu, 0x4au, 0x57u, 0x70u, 0x6du, 0x3eu, 0x23u,
  0x04u, 0x19u, 0xa2u, 0xbfu, 0x98u, 0x85u, 0xd6u, 0xcbu, 0xecu,
  0xf1u, 0x13u, 0x0eu, 0x29u, 0x34u, 0x67u, 0x7au, 0x5du, 0x40u,
  0xfbu, 0xe6u, 0xc1u, 0xdcu, 0x8fu, 0x92u, 0xb5u, 0xa8u, 0xdeu,
  0xc3u, 0xe4u, 0xf9u, 0xaau, 0xb7u, 0x90u, 0x8du, 0x36u, 0x2bu,
  0x0cu, 0x11u, 0x42u, 0x5fu, 0x78u, 0x65u, 0x94u, 0x89u, 0xaeu,
  0xb3u, 0xe0u, 0xfdu, 0xdau, 0xc7u, 0x7cu, 0x61u, 0x46u, 0x5bu,
  0x08u, 0x15u, 0x32u, 0x2fu, 0x59u, 0x44u, 0x63u, 0x7eu, 0x2du,
  0x30u, 0x17u, 0x0au, 0xb1u, 0xacu, 0x8bu, 0x96u, 0xc5u, 0xd8u,
  0xffu, 0xe2u, 0x26u, 0x3bu, 0x1cu, 0x01u, 0x52u, 0x4fu, 0x68u,
  0x75u, 0xceu, 0xd3u, 0xf4u, 0xe9u, 0xbau, 0xa7u, 0x80u, 0x9du,
  0xebu, 0xf6u, 0xd1u, 0xccu, 0x9fu, 0x82u, 0xa5u, 0xb8u, 0x03u,
  0x1eu, 0x39u, 0x24u, 0x77u, 0x6au, 0x4du, 0x50u, 0xa1u, 0xbcu,
  0x9bu, 0x86u, 0xd5u, 0xc8u, 0xefu, 0xf2u, 0x49u, 0x54u, 0x73u,
  0x6eu, 0x3du, 0x20u, 0x07u, 0x1au, 0x6cu, 0x71u, 0x56u, 0x4bu,
  0x18u, 0x05u, 0x22u, 0x3fu, 0x84u, 0x99u, 0xbeu, 0xa3u, 0xf0u,
  0xedu, 0xcau, 0xd7u, 0x35u, 0x28u, 0x0fu, 0x12u, 0x41u, 0x5cu,
  0x7bu, 0x66u, 0xddu, 0xc0u, 0xe7u, 0xfau, 0xa9u, 0xb4u, 0x93u,
  0x8eu, 0xf8u, 0xe5u, 0xc2u, 0xdfu, 0x8cu, 0x91u, 0xb6u, 0xabu,
  0x10u, 0x0du, 0x2au, 0x37u, 0x64u, 0x79u, 0x5eu, 0x43u, 0xb2u,
  0xafu, 0x88u, 0x95u, 0xc6u, 0xdbu, 0xfcu, 0xe1u, 0x5au, 0x47u,
  0x60u, 0x7du, 0x2eu, 0x33u, 0x14u, 0x09u, 0x7fu, 0x62u, 0x45u,
  0x58u, 0x0bu, 0x16u, 0x31u, 0x2cu, 0x97u, 0x8au, 0xadu, 0xb0u,
  0xe3u, 0xfeu, 0xd9u, 0xc4u
};

fs26_watchdog_type wd_type = FS26_WD_CHALLENGER;
static uint16_t watchdog_token = 0x5ab2;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline void fs26_configspi(struct spi_dev_s *spi)
{
  /* Configure SPI for the ADXL345 */

  SPI_SETMODE(spi, SPIDEV_MODE1); /* CPOL=0 CPHA=1 */
  SPI_SETBITS(spi, 32);
  SPI_SETFREQUENCY(spi, CONFIG_FS26_SPI_FREQUENCY);
}

static inline void fs26_print_general_device_status(uint32_t retval)
{
  if (retval & FS26_M_AVAL)
    {
      spierr("Main State machine availability\n");
    }
  if (retval & FS26_FS_EN)
    {
      spierr("Fail Safe State machine status enabled\n");
    }

  spierr("IRQ:\n");
  if (retval & FS26_FS_G)
    {
      spierr("FS26_FS_G\n");
    }
  if (retval & FS26_COM_G)
    {
      spierr("FS26_COM_G\n");
    }
  if (retval & FS26_WIO_G)
    {
      spierr("FS26_WIO_G\n");
    }
  if (retval & FS26_VSUP_G)
    {
      spierr("FS26_VSUP_G\n");
    }
  if (retval & FS26_REG_G)
    {
      spierr("FS26_REG_G\n");
    }
  if (retval & FS26_TSD_G)
    {
      spierr("FS26_TSD_G\n");
    }
}

static inline void fs26_print_crc(uint8_t crc, uint8_t crc_received)
{
  spierr("%02x<->%02x\n", crc, crc_received);
}

/* Computes Challenger Watchdog answer. */

static inline uint16_t fs26_wdcomputeanswer(uint16_t token)
{
  uint32_t u32_mr = token; /* Monitoring result. */

  /* Simulates ALU Checker on the MCU side. */

  u32_mr *= 4U;
  u32_mr += 6U;
  u32_mr -= 4U;
  u32_mr = ~u32_mr;
  u32_mr /= 4U;

  return (uint16_t)u32_mr;
}

static uint8_t fs26_calcrc(const uint8_t * data, uint8_t datalen)
{
  uint8_t crc;      /* Result. */
  uint8_t tableidx; /* Index to the CRC table. */
  uint8_t dataidx;  /* Index to the data array (memory). */

  DEBUGASSERT(data != NULL);
  DEBUGASSERT(datalen > 0);

  /* Set CRC token value. */

  crc = FS26_COM_CRC_INIT;

  for (dataidx = datalen; dataidx > 0; dataidx--)
    {
      tableidx = crc ^ data[dataidx];
      crc = FS26_CRC_TABLE[tableidx];
    }

  return crc;
}

uint32_t fs26_setreg(struct fs26_dev_s *priv, uint8_t regaddr,
                     uint16_t regval)
{
  uint32_t retval;
  uint32_t spidata;

  /* Send register address and set the value */

  spidata = FS26_REG_ADDR(regaddr) | FS26_SET_DATA(regval) | FS26_RW;
  spidata |= (uint32_t)fs26_calcrc((uint8_t *)&spidata, 3);

#ifdef SWAP_ENDIANESS
  spidata = __builtin_bswap32(spidata);
#endif

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  fs26_configspi(priv->spi);

  /* Select the FS26 */

  SPI_SELECT(priv->spi, SPIDEV_NONE(0), true);

  SPI_EXCHANGE(priv->spi, &spidata, &retval, 1);

#ifdef SWAP_ENDIANESS
  retval = __builtin_bswap32(retval);
#endif

  spiinfo("Received %08lx\n", retval);

  if (fs26_calcrc((uint8_t *)&retval, 3) != ((uint8_t *)&retval)[0])
    {
      spierr("CRC error expected %02x got %02x\n",
                fs26_calcrc((uint8_t *)&retval, 3),
                ((uint8_t *)&retval)[0]);
      retval = 0x0;
    }

  /* Deselect the FS26 */

  SPI_SELECT(priv->spi, SPIDEV_NONE(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  return retval;
}

uint32_t fs26_getreg(struct fs26_dev_s *priv, uint8_t regaddr)
{
  uint32_t retval;
  uint32_t spidata;

  /* Send register address and calc CRC */

  spidata  = FS26_REG_ADDR(regaddr);
  spidata |= (uint32_t)fs26_calcrc((uint8_t *)&spidata, 3);

#ifdef SWAP_ENDIANESS
  spidata = __builtin_bswap32(spidata);
#endif

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  fs26_configspi(priv->spi);

  /* Select the FS26 */

  SPI_SELECT(priv->spi, SPIDEV_NONE(0), true);

  /* Send register to read and get the 32 bits */

  SPI_EXCHANGE(priv->spi, &spidata, &retval, 1);

#ifdef SWAP_ENDIANESS
  retval = __builtin_bswap32(retval);
#endif

  if (fs26_calcrc((uint8_t *)&retval, 3) != ((uint8_t *)&retval)[0])
    {
      spierr("CRC error expected %02x got %02x\n",
                fs26_calcrc((uint8_t *)&retval, 3),
                ((uint8_t *)&retval)[0]);
      retval = 0x0;
    }

  /* Deselect the FS26 */

  SPI_SELECT(priv->spi, SPIDEV_NONE(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  /* DEBUG print */

  spiinfo("%02x->%04lx\n", regaddr, retval);

  return retval;
}

static uint32_t fs26_wdreadchallengetoken(uint16_t * token_ptr)
{
  uint32_t retval;

  retval = fs26_getreg(&g_fs26, FS26_FS_WD_TOKEN);
  if (retval != 0)
    {
      *token_ptr = FS26_GET_DATA(retval);
      retval = OK;
    }

  return retval;
}

int32_t fs26_wdrefresh(void)
{
    int32_t retval = OK;
    uint16_t u16_answer; /* Calculated monitoring result. */
    irqstate_t flags;

    if (FS26_WD_DISABLED == wd_type)
      {
        /* No need to refresh watchdog. */

        retval = OK;
      }
    else if(FS26_WD_SIMPLE == wd_type)
      {
        if (fs26_setreg(&g_fs26, FS26_FS_WD_ANSWER, watchdog_token) != 0)
          {
            retval = OK;
          }
        else
          {
            retval = -1;
          }
      }
    else if(FS26_WD_CHALLENGER == wd_type)
      {
        flags = enter_critical_section();

        if (fs26_wdreadchallengetoken(&watchdog_token) == OK)
          {
            u16_answer = fs26_wdcomputeanswer(watchdog_token);

            if (fs26_setreg(&g_fs26, FS26_FS_WD_ANSWER, u16_answer) != 0)
            {
                retval = OK;
            }
            else
            {
                retval = -1;
            }
          }
        else
          {
              retval = -1;
          }

        leave_critical_section(flags);
      }

    /* Check if watchdog refresh was successful. */

    if (((FS26_WD_SIMPLE == wd_type)
         || (FS26_WD_CHALLENGER == wd_type))
         && (OK == retval))
      {
          if ((FS26_GET_DATA(
              fs26_getreg(&g_fs26, FS26_FS_GRL_FLAGS))
               & FS_WD_G_MASK) == FS_WD_G)
            {
                retval = -1;
            }
          else
            {
                retval = OK;
            }
      }
    return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void fs26_initialize(struct spi_dev_s *spi)
{
  uint32_t retval;
  uint16_t regval;

  /* Assign spi controller */

  g_fs26.spi = spi;

  /* Check FS diag */

  retval = fs26_getreg(&g_fs26, FS26_FS_DIAG_SAFETY1);

  if ((FS26_GET_DATA(retval) & (ABIST1_PASS_MASK | LBIST_STATUS_MASK))
      != (ABIST1_PASS | LBIST_STATUS_OK))
    {
      spierr("FS26 DIAG failed %08lx\n", retval);
    }

  /* Get state machine state */

  retval = fs26_getreg(&g_fs26, FS26_FS_STATES);

  if ((FS26_GET_DATA(retval) & DBG_MODE_MASK) == DBG_MODE)
    {
      spierr("FS26 in DEBUG mode\n");
    }

  /* INIT_FS */

  if ((FS26_GET_DATA(retval) & FS_STATES_MASK) == FS_STATES_INIT_FS)
    {
      /* Set all FS26_FS_I_XXX registers
       * Note write data to both normal and not registers
       */

      regval = VMON_PRE_OV_FS_REACTION_NO_EFFECT |
                VMON_PRE_UV_FS_REACTION_NO_EFFECT |
                VMON_CORE_OV_FS_REACTION_NO_EFFECT |
                VMON_CORE_UV_FS_REACTION_NO_EFFECT |
                VMON_LDO1_OV_FS_REACTION_NO_EFFECT |
                VMON_LDO1_UV_FS_REACTION_NO_EFFECT |
                VMON_LDO2_OV_FS_REACTION_NO_EFFECT |
                VMON_LDO2_UV_FS_REACTION_NO_EFFECT;

      fs26_setreg(&g_fs26, FS26_FS_I_OVUV_SAFE_REACTION1, regval);
      fs26_setreg(&g_fs26, FS26_FS_I_NOT_OVUV_SAFE_REACTION1, ~regval);

      regval = VMON_EXT_OV_FS_REACTION_NO_EFFECT |
                VMON_EXT_UV_FS_REACTION_NO_EFFECT |
                VMON_REF_OV_FS_REACTION_NO_EFFECT |
                VMON_REF_UV_FS_REACTION_NO_EFFECT |
                VMON_TRK2_OV_FS_REACTION_NO_EFFECT |
                VMON_TRK2_UV_FS_REACTION_NO_EFFECT |
                VMON_TRK1_OV_FS_REACTION_NO_EFFECT |
                VMON_TRK1_UV_FS_REACTION_NO_EFFECT;

      fs26_setreg(&g_fs26, FS26_FS_I_OVUV_SAFE_REACTION2, regval);
      fs26_setreg(&g_fs26, FS26_FS_I_NOT_OVUV_SAFE_REACTION2, ~regval);

      regval = WD_ERR_LIMIT_8 | WD_RFR_LIMIT_6 | WD_FS_REACTION_NO_ACTION;

      fs26_setreg(&g_fs26, FS26_FS_I_WD_CFG, regval);
      fs26_setreg(&g_fs26, FS26_FS_I_NOT_WD_CFG, ~regval);

      regval = FCCU_CFG_NO_MONITORING | ERRMON_ACK_TIME_32MS;

      fs26_setreg(&g_fs26, FS26_FS_I_SAFE_INPUTS, regval);
      fs26_setreg(&g_fs26, FS26_FS_I_NOT_SAFE_INPUTS, ~regval);

      regval = FLT_ERR_REACTION_NO_EFFECT | CLK_MON_DIS | DIS8S;

      fs26_setreg(&g_fs26, FS26_FS_I_FSSM, regval);
      fs26_setreg(&g_fs26, FS26_FS_I_NOT_FSSM, ~regval);

      /* Disable watchdog */

      regval = WDW_PERIOD_DISABLE | WDW_DC_62_37 | WDW_RECOVERY_DISABLE;

      fs26_setreg(&g_fs26, FS26_FS_WDW_DURATION, regval);
      fs26_setreg(&g_fs26, FS26_FS_NOT_WDW_DURATION, ~regval);

      fs26_wdrefresh();

      spierr("FS26 in INIT_FS mode\n");
    }
  else if ((FS26_GET_DATA(retval) & FS_STATES_MASK) == FS_STATES_DEBUG_ENTRY)
    {
      spierr("FS26 in DEBUG_ENTRY mode\n");
    }
  else if ((FS26_GET_DATA(retval) & FS_STATES_MASK) == FS_STATES_NORMAL)
    {
      spierr("FS26 in NORMAL mode\n");
    }
  else if ((FS26_GET_DATA(retval) & FS_STATES_MASK)
           == FS_STATES_SAFETY_OUT_NOT)
    {
      spierr("FS26 in Safety Outputs not released\n");
    }
}

#endif /* CONFIG_S32K3XX_FS26 */
