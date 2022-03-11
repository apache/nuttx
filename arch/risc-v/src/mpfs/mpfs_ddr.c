/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_ddr.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/signal.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>

#include <arch/board/board.h>
#include <arch/board/board_liberodefs.h>

#include "riscv_internal.h"
#include "mpfs_dma.h"
#include "hardware/mpfs_sysreg.h"
#include "hardware/mpfs_ddr.h"
#include "hardware/mpfs_sgmii.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DDR write latency range */

#define WR_LATENCY_MIN                  0
#define WR_LATENCY_MAX                  3

/* Training types status offsets */

#define BCLK_SCLK_BIT                   (1 << 0)
#define ADDCMD_BIT                      (1 << 1)
#define WRLVL_BIT                       (1 << 2)
#define RDGATE_BIT                      (1 << 3)
#define DQ_DQS_BIT                      (1 << 4)

#define ONE_MB_MTC                      20

/* RPC values and ranges */

#define DEFAULT_RPC_166_VALUE           2
#define MIN_RPC_166_VALUE               2
#define MAX_RPC_166_VALUE               4
#define NUM_RPC_166_VALUES              (MAX_RPC_166_VALUE - \
                                         MIN_RPC_166_VALUE)

/* Write - read memory test patterns */

#define PATTERN_INCREMENTAL             (0x01 << 0)
#define PATTERN_WALKING_ONE             (0x01 << 1)
#define PATTERN_WALKING_ZERO            (0x01 << 2)
#define PATTERN_RANDOM                  (0x01 << 3)
#define PATTERN_CCCCCCCC                (0x01 << 4)
#define PATTERN_55555555                (0x01 << 5)
#define PATTERN_ZEROS                   (0x01 << 6)
#define MAX_NO_PATTERNS                 7

/* Write - read memory test properties */

#define SW_CFG_NUM_READS_WRITES         0x20000
#define SW_CONFIG_PATTERN (PATTERN_INCREMENTAL  | \
                           PATTERN_WALKING_ONE  | \
                           PATTERN_WALKING_ZERO | \
                           PATTERN_RANDOM       | \
                           PATTERN_CCCCCCCC     | \
                           PATTERN_55555555)

/* Retraining limits */

#define ABNORMAL_RETRAIN_CA_DECREASE_COUNT          2
#define ABNORMAL_RETRAIN_CA_DLY_DECREASE_COUNT      2
#define DQ_DQS_NUM_TAPS                             5

/* PLL convenience bits */

#define PLL_CTRL_LOCK_BIT               (1 << 25)
#define PLL_INIT_AND_OUT_OF_RESET       0x00000003
#define PLL_CTRL_REG_POWERDOWN_B_MASK   0x00000001

/* DDRPHY bit masks and defines */

#define DDRPHY_MODE_BUS_WIDTH_MASK      (0x7 << 5)
#define DDRPHY_MODE_BUS_WIDTH_4_LANE    (0x1 << 5)
#define DDRPHY_MODE_MASK                0x7
#define DDRPHY_MODE_ECC_MASK            (0x1 << 3)
#define DDRPHY_MODE_ECC_ON              (0x1 << 3)
#define DDRPHY_MODE_RANK_MASK           (0x1 << 26)
#define DDRPHY_MODE_ONE_RANK            (0x0 << 26)
#define DDRPHY_MODE_TWO_RANKS           (0x1 << 26)
#define DMI_DBI_MASK                    (~(0x1 << 8))

/* DDR frequency properties */

#define DDR_FREQ_MARGIN                 10
#define DDR_1333_MHZ                    1333333333
#define DDR_1600_MHZ                    1600000000

/* MPFS clock configuration register */

#define MPFS_SYSREG_SOFT_RESET_CR       (MPFS_SYSREG_BASE + \
                                         MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR     (MPFS_SYSREG_BASE + \
                                         MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

/* Max supported lanes */

#define MAX_LANES                       5

/* BCLK / SCLK offset */

#define SW_TRAINING_BCLK_SCLK_OFFSET    0x00000000

/* Register poll retry counts until fail */

#define MPFS_DEFAULT_RETRIES            0xffff
#define MPFS_LONG_RETRIES               0xffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum ddr_type_e
{
    DDR3                                = 0x00,
    DDR3L                               = 0x01,
    DDR4                                = 0x02,
    LPDDR3                              = 0x03,
    LPDDR4                              = 0x04,
    DDR_OFF_MODE                        = 0x07
};

enum seg_setup_e
{
    DEFAULT_SEG_SETUP                   = 0x00,
    LIBERO_SEG_SETUP
};

enum mtc_pattern_e
{
    MTC_COUNTING_PATTERN                = 0x00,
    MTC_WALKING_ONE                     = 0x01,
    MTC_PSEUDO_RANDOM                   = 0x02,
    MTC_NO_REPEATING_PSEUDO_RANDOM      = 0x03,
    MTC_ALT_ONES_ZEROS                  = 0x04,
    MTC_ALT_5_A                         = 0x05,
    MTC_USER                            = 0x06,
    MTC_PSEUDO_RANDOM_16BIT             = 0x07,
    MTC_PSEUDO_RANDOM_8BIT              = 0x08,
};

enum mtc_add_pattern_e
{
    MTC_ADD_SEQUENTIAL                  = 0x00,
    MTC_ADD_RANDOM                      = 0x01,
};

enum ddr_access_size_e
{
    DDR_32_BIT,
    DDR_64_BIT
};

typedef struct
{
  uint32_t status_lower;
  uint32_t status_upper;
  uint32_t lower;
  uint32_t upper;
  uint32_t vref_result;
} mss_ddr_vref_t;

typedef struct
{
  uint32_t status_lower;
  uint32_t lower[MAX_LANES];
  uint32_t lane_calib_result;
} mss_mpfs_ddr_write_calibration_t;

typedef struct
{
  uint32_t lower[MAX_LANES];
  uint32_t upper[MAX_LANES];
  uint32_t calibration_found[MAX_LANES];
} mss_lpddr4_dq_calibration_t;

typedef struct
{
  mss_mpfs_ddr_write_calibration_t write_cal;
  mss_lpddr4_dq_calibration_t      dq_cal;
  mss_ddr_vref_t                   fpga_vref;
  mss_ddr_vref_t                   mem_vref;
} mss_ddr_calibration_t;

struct mpfs_ddr_priv_s
{
  enum ddr_type_e        ddr_type;
  int                    error;
  uint32_t               timeout;
  uint32_t               retry_count;
  uint32_t               write_latency;
  uint32_t               tip_cfg_params;
  uint32_t               dpc_bits;
  uint32_t               rpc_166_fifo_offset;
  uint8_t                last_sweep_status;
  uint8_t                num_rpc_166_retires;
  uint32_t               bclk_answer;
  uint32_t               ret_status;
  uint8_t                number_of_lanes_to_calibrate;
  uint8_t                refclk_sweep_index;
  bool                   en_addcmd0_ovrt9;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mss_ddr_calibration_t calib_data;

static struct mpfs_ddr_priv_s g_mpfs_ddr_priv =
{
  .en_addcmd0_ovrt9     = true,
  .refclk_sweep_index   = 0xf,
  .bclk_answer          = 0,
};

static const uint8_t refclk_offsets[][5] =
  {
    {LIBERO_SETTING_REFCLK_DDR3_1333_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_3},
    {LIBERO_SETTING_REFCLK_DDR3L_1333_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_3},
    {LIBERO_SETTING_REFCLK_DDR4_1600_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_3},
    {LIBERO_SETTING_REFCLK_LPDDR3_1600_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_0,
     LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_1,
     LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_2,
     LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_3},
    {LIBERO_SETTING_REFCLK_LPDDR4_1600_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_0,
     LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_1,
     LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_2,
     LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_3},

    {LIBERO_SETTING_REFCLK_DDR3_1067_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_3},
    {LIBERO_SETTING_REFCLK_DDR3L_1067_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_3},
    {LIBERO_SETTING_REFCLK_DDR4_1333_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_0,
     LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_1,
     LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_2,
     LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_3},
    {LIBERO_SETTING_REFCLK_LPDDR3_1333_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_0,
     LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_1,
     LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_2,
     LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_3},
    {LIBERO_SETTING_REFCLK_LPDDR4_1333_NUM_OFFSETS,
     LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_0,
     LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_1,
     LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_2,
     LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_3},
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_wait_cycles
 *
 * Description:
 *   Wait n number of cycles. This mimics the vendor provided wait loops.
 *
 * Input Parameters:
 *   n   - number of cycles to loop
 *
 ****************************************************************************/

static void mpfs_wait_cycles(uint32_t n)
{
  volatile uint32_t count = n;

  while (count != 0)
    {
        count--;
    }
}

/****************************************************************************
 * Name: mpfs_ddr_pll_config_scb_turn_off
 *
 * Description:
 *   Turns off the PLL
 *
 ****************************************************************************/

static void mpfs_ddr_pll_config_scb_turn_off(void)
{
  modifyreg32(MPFS_IOSCB_MSS_PLL_CTRL, 0x01, 0);
}

/****************************************************************************
 * Name: mpfs_ddr_off_mode
 *
 * Description:
 *   Sets the DDR in off mode
 *
 ****************************************************************************/

static void mpfs_ddr_off_mode(void)
{
  /* Sets the mode register to off mode */

  putreg32(LIBERO_SETTING_DDRPHY_MODE_OFF,
           MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE);

  /* VS for off mode */

  putreg32(LIBERO_SETTING_DPC_BITS_OFF_MODE,
           MPFS_CFG_DDR_SGMII_PHY_DPC_BITS);

  /* Toggle decoder here */

  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_DRIVER);
  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_ODT);
  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_IO);

  /* Set ibuff to off mode */

  putreg32(0x07, MPFS_CFG_DDR_SGMII_PHY_RPC95); /* addcmd I/O */
  putreg32(0x07, MPFS_CFG_DDR_SGMII_PHY_RPC96); /* clk */
  putreg32(0x07, MPFS_CFG_DDR_SGMII_PHY_RPC97); /* dq */
  putreg32(0x07, MPFS_CFG_DDR_SGMII_PHY_RPC98); /* dqs */

  /* SPARE_0:
   *   bits 15:14 connect to ibufmx DQ/DQS/DM
   *   bits 13:12 connect to ibufmx CA/CK
   */

  putreg32(0, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);

  /* REG_POWERDOWN_B on PLL turn-off, in case was turned on */

  mpfs_ddr_pll_config_scb_turn_off();
}

/****************************************************************************
 * Name: mpfs_set_ddr_mode_reg_and_vs_bits
 *
 * Description:
 *   Sets the DDR PHY mode.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure.
 *
 ****************************************************************************/

static void mpfs_set_ddr_mode_reg_and_vs_bits(struct mpfs_ddr_priv_s *priv)
{
  if ((priv->ddr_type == DDR4) &&
      (LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_ECC_MASK) ==
       DDRPHY_MODE_ECC_ON)
    {
      /* For ECC on when DDR4, and data mask on during training, training
       * will not pass. This will eventually be handled by the configurator.
       * DM will not be allowed for DDR4 with ECC.
       */

      putreg32((LIBERO_SETTING_DDRPHY_MODE & DMI_DBI_MASK),
               MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE);
    }
  else
    {
      putreg32((LIBERO_SETTING_DDRPHY_MODE),
               MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE);
    }

  mpfs_wait_cycles(100);

  putreg32(priv->dpc_bits, MPFS_CFG_DDR_SGMII_PHY_DPC_BITS);
}

/****************************************************************************
 * Name: mpfs_config_ddr_io_pull_up_downs
 *
 * Description:
 *   Configure DDR pull-ups and pull-downs.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure.
 *
 ****************************************************************************/

static void mpfs_config_ddr_io_pull_up_downs(struct mpfs_ddr_priv_s *priv)
{
  if (priv->ddr_type == LPDDR4 && priv->en_addcmd0_ovrt9)
    {
      /* set overrides (associated bit set to 1 if I/O not being used */

      putreg32(LIBERO_SETTING_RPC_EN_ADDCMD0_OVRT9,
               MPFS_CFG_DDR_SGMII_PHY_OVRT9);
      putreg32(LIBERO_SETTING_RPC_EN_ADDCMD1_OVRT10,
               MPFS_CFG_DDR_SGMII_PHY_OVRT10);
      putreg32(LIBERO_SETTING_RPC_EN_ADDCMD2_OVRT11,
               MPFS_CFG_DDR_SGMII_PHY_OVRT11);
      putreg32(LIBERO_SETTING_RPC_EN_DATA0_OVRT12,
               MPFS_CFG_DDR_SGMII_PHY_OVRT12);
      putreg32(LIBERO_SETTING_RPC_EN_DATA1_OVRT13,
               MPFS_CFG_DDR_SGMII_PHY_OVRT13);
      putreg32(LIBERO_SETTING_RPC_EN_DATA2_OVRT14,
               MPFS_CFG_DDR_SGMII_PHY_OVRT14);
      putreg32(LIBERO_SETTING_RPC_EN_DATA3_OVRT15,
               MPFS_CFG_DDR_SGMII_PHY_OVRT15);
      putreg32(LIBERO_SETTING_RPC_EN_ECC_OVRT16,
               MPFS_CFG_DDR_SGMII_PHY_OVRT16);

      /* Set the required wpu state.
       * Note: associated I/O bit 1=> off, 0=> on
       */

      putreg32(LIBERO_SETTING_RPC235_WPD_ADD_CMD0,
               MPFS_CFG_DDR_SGMII_PHY_RPC235);
      putreg32(LIBERO_SETTING_RPC236_WPD_ADD_CMD1,
               MPFS_CFG_DDR_SGMII_PHY_RPC236);
      putreg32(LIBERO_SETTING_RPC237_WPD_ADD_CMD2,
               MPFS_CFG_DDR_SGMII_PHY_RPC237);
      putreg32(LIBERO_SETTING_RPC238_WPD_DATA0,
               MPFS_CFG_DDR_SGMII_PHY_RPC238);
      putreg32(LIBERO_SETTING_RPC239_WPD_DATA1,
               MPFS_CFG_DDR_SGMII_PHY_RPC239);
      putreg32(LIBERO_SETTING_RPC240_WPD_DATA2,
               MPFS_CFG_DDR_SGMII_PHY_RPC240);
      putreg32(LIBERO_SETTING_RPC241_WPD_DATA3,
               MPFS_CFG_DDR_SGMII_PHY_RPC241);
      putreg32(LIBERO_SETTING_RPC242_WPD_ECC,
               MPFS_CFG_DDR_SGMII_PHY_RPC242);

      /* Set the required wpd state.
       * Note: associated I/O bit 1=> off, 0=> on
       */

      putreg32(LIBERO_SETTING_RPC243_WPU_ADD_CMD0,
               MPFS_CFG_DDR_SGMII_PHY_RPC243);
      putreg32(LIBERO_SETTING_RPC244_WPU_ADD_CMD1,
               MPFS_CFG_DDR_SGMII_PHY_RPC244);
      putreg32(LIBERO_SETTING_RPC245_WPU_ADD_CMD2,
               MPFS_CFG_DDR_SGMII_PHY_RPC245);
      putreg32(LIBERO_SETTING_RPC246_WPU_DATA0,
               MPFS_CFG_DDR_SGMII_PHY_RPC246);
      putreg32(LIBERO_SETTING_RPC247_WPU_DATA1,
               MPFS_CFG_DDR_SGMII_PHY_RPC247);
      putreg32(LIBERO_SETTING_RPC248_WPU_DATA2,
               MPFS_CFG_DDR_SGMII_PHY_RPC248);
      putreg32(LIBERO_SETTING_RPC249_WPU_DATA3,
               MPFS_CFG_DDR_SGMII_PHY_RPC249);
      putreg32(LIBERO_SETTING_RPC250_WPU_ECC,
               MPFS_CFG_DDR_SGMII_PHY_RPC250);
    }
}

/****************************************************************************
 * Name: mpfs_set_ddr_rpc_regs
 *
 * Description:
 *   Write DDR phy mode reg (eg. DDR3).
 *   When we write to the mode register, an ip state machine copies default
 *   values for the particular mode chosen to RPC registers associated with
 *   DDR in the MSS custom block.
 *
 *   The RPC register values are transferred to the SCB registers in a
 *   subsequent step.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure.
 *
 ****************************************************************************/

static void mpfs_set_ddr_rpc_regs(struct mpfs_ddr_priv_s *priv)
{
  switch (priv->ddr_type)
    {
      default:
      case DDR_OFF_MODE:
        putreg32(0, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);
        break;

      case DDR3L:
      case DDR3:

        /* Required when rank x 2 */

        if ((LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_RANK_MASK) ==
            DDRPHY_MODE_TWO_RANKS)
          {
            putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SPIO253);
          }

        putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_RPC98);
        putreg32(0, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);
        break;

      case DDR4:
        putreg32(2, MPFS_CFG_DDR_SGMII_PHY_RPC10_ODT);
        putreg32(2, MPFS_CFG_DDR_SGMII_PHY_RPC11_ODT);
        putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_RPC98);
        putreg32(0, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);
        break;

      case LPDDR3:
        putreg32(2, MPFS_CFG_DDR_SGMII_PHY_RPC10_ODT);
        putreg32(2, MPFS_CFG_DDR_SGMII_PHY_RPC11_ODT);
        putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_RPC98);
        putreg32(0, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);
        break;

      case LPDDR4:

        /* OVRT_EN_ADDCMD1 (default 0xf00), register named ovrt11 */

        if (!priv->en_addcmd0_ovrt9)
          {
            /* If this define is not present, indicates older
             * Libero core (pre 2.0.109), so we run this code.
             */

            putreg32(LIBERO_SETTING_RPC_EN_ADDCMD1_OVRT10,
                     MPFS_CFG_DDR_SGMII_PHY_OVRT10);

            /* Use pull-ups to set the CMD/ADD ODT */

            putreg32(0, MPFS_CFG_DDR_SGMII_PHY_RPC245);
            putreg32(0xffffffff, MPFS_CFG_DDR_SGMII_PHY_RPC237);

            /* OVRT_EN_ADDCMD2 (default 0xE06U), register named ovrt12 */

            putreg32(LIBERO_SETTING_RPC_EN_ADDCMD2_OVRT11,
                     MPFS_CFG_DDR_SGMII_PHY_OVRT11);
          }

            /* Required when rank x 2 */

          if ((LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_RANK_MASK) ==
              DDRPHY_MODE_TWO_RANKS)
            {
              putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SPIO253);
            }

        putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_RPC98);
        putreg32(0xa000, MPFS_CFG_DDR_SGMII_PHY_SPARE_0);
        break;
    }

  putreg32(0x2, MPFS_CFG_DDR_SGMII_PHY_RPC27);
  putreg32(0, MPFS_CFG_DDR_SGMII_PHY_RPC203);

  putreg32(LIBERO_SETTING_RPC_ODT_ADDCMD, MPFS_CFG_DDR_SGMII_PHY_RPC1_ODT);
  putreg32(LIBERO_SETTING_RPC_ODT_CLK, MPFS_CFG_DDR_SGMII_PHY_RPC2_ODT);
  putreg32(LIBERO_SETTING_RPC_ODT_DQ, MPFS_CFG_DDR_SGMII_PHY_RPC3_ODT);
  putreg32(LIBERO_SETTING_RPC_ODT_DQS, MPFS_CFG_DDR_SGMII_PHY_RPC4_ODT);

  /* bclk_sel_clkn - selects bclk sclk training clock */

  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_RPC19);

  /* Add cmd - selects bclk sclk training clock */

  putreg32(0, MPFS_CFG_DDR_SGMII_PHY_RPC20);

  /* Each lane has its own FIFO. This paramater adjusts offset for
   * all lanes.
   */

  putreg32(priv->rpc_166_fifo_offset, MPFS_CFG_DDR_SGMII_PHY_RPC166);

  mpfs_config_ddr_io_pull_up_downs(priv);
}

/****************************************************************************
 * Name: mpfs_ddr_pvt_calibration
 *
 * Description:
 *   PVT calibration:
 *     Wait for IOEN from power detectors DDR and SGMII - IO enable signal
 *     from System Control power on.
 *
 * Returned Value:
 *   0 on success, -ETIMEDOUT in case of error.
 *
 ****************************************************************************/

static int mpfs_ddr_pvt_calibration(void)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  while ((!(getreg32(MPFS_CFG_DDR_SGMII_PHY_IOC_REG1) & (1 << 4))) &&
         --retries);

  if (retries == 0)
    {
      merr("timeout!\n");
      return -ETIMEDOUT;
    }

  /* Voltage ramp time */

  mpfs_wait_cycles(0xf);

  /* DDRIO:  calib_reset: 1 -> 0, clk divider changed - from 2 - to 3 */

  putreg32(0x06, MPFS_CFG_DDR_SGMII_PHY_IOC_REG6);

  /* PVT soft nv reset - SCB, should load from RPC */

  putreg32(1, MPFS_IOSCB_CALIB_DDR_SOFT_RESET);
  putreg32(0, MPFS_IOSCB_CALIB_DDR_SOFT_RESET);

  /* R3.4 Wait for PVT calibration to complete;
   * Check bit 2 sro_calib_status.
   *
   * The G5 Memory controller needs to see that the IO calibration has
   * completed before kicking off DDR training.
   * It uses the calib_status signal as a flag for this.
   */

  while (!(getreg32(MPFS_IOSCB_CALIB_DDR_IOC_REG1) & 0x04));

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_IOC_REG1) & 0x04));

  /* Assert calib lock */

  modifyreg32(MPFS_CFG_DDR_SGMII_PHY_IOC_REG0, (1 << 14), 0);
  modifyreg32(MPFS_IOSCB_CALIB_DDR_IOC_REG0, (1 << 14), 0);
  modifyreg32(MPFS_CFG_DDR_SGMII_PHY_IOC_REG0, 0, (1 << 14));
  modifyreg32(MPFS_IOSCB_CALIB_DDR_IOC_REG0, 0, (1 << 14));

  return 0;
}

/****************************************************************************
 * Name: mpfs_ddr_pll_config
 *
 * Description:
 *   Configure DDR PLL according to a set of predefined values.
 *
 ****************************************************************************/

static void mpfs_ddr_pll_config(void)
{
  putreg32(PLL_INIT_AND_OUT_OF_RESET, MPFS_IOSCB_DDR_PLL_SOFT_RESET);
  putreg32(LIBERO_SETTING_DDR_PLL_CTRL &
           ~(PLL_CTRL_REG_POWERDOWN_B_MASK), MPFS_IOSCB_DDR_PLL_CTRL);

  putreg32(LIBERO_SETTING_DDR_PLL_REF_FB, MPFS_IOSCB_DDR_PLL_REF_FB);

  putreg32(LIBERO_SETTING_DDR_PLL_DIV_0_1, MPFS_IOSCB_DDR_PLL_DIV_0_1);
  putreg32(LIBERO_SETTING_DDR_PLL_DIV_2_3, MPFS_IOSCB_DDR_PLL_DIV_2_3);
  putreg32(LIBERO_SETTING_DDR_PLL_CTRL2, MPFS_IOSCB_DDR_PLL_CTRL2);
  putreg32(LIBERO_SETTING_DDR_PLL_FRACN, MPFS_IOSCB_DDR_PLL_FRACN);

  putreg32(LIBERO_SETTING_DDR_SSCG_REG_0, MPFS_IOSCB_DDR_PLL_SSCG_REG_0);
  putreg32(LIBERO_SETTING_DDR_SSCG_REG_1, MPFS_IOSCB_DDR_PLL_SSCG_REG_1);
  putreg32(LIBERO_SETTING_DDR_SSCG_REG_2, MPFS_IOSCB_DDR_PLL_SSCG_REG_2);
  putreg32(LIBERO_SETTING_DDR_SSCG_REG_3, MPFS_IOSCB_DDR_PLL_SSCG_REG_3);

  putreg32(LIBERO_SETTING_DDR_PLL_PHADJ, MPFS_IOSCB_DDR_PLL_PHADJ);
  putreg32(LIBERO_SETTING_DDR_PLL_CTRL | 1, MPFS_IOSCB_DDR_PLL_CTRL);
}

/****************************************************************************
 * Name: mpfs_ddr_pll_lock_scb
 *
 * Description:
 *   Check whether the DDR PLL has locked.
 *
 * Returned Value:
 *   0 if PLL has locked, 1 otherwise.
 *
 ****************************************************************************/

static int mpfs_ddr_pll_lock_scb(void)
{
  if (getreg32(MPFS_IOSCB_DDR_PLL_CTRL) & PLL_CTRL_LOCK_BIT)
    {
      /* PLL has locked */

      return 0;
    }

  return 1;
}

#define INIT_SETTING_SEG0_0    0x00007F80
#define INIT_SETTING_SEG0_1    0x00007000
#define INIT_SETTING_SEG0_2    0x00000000
#define INIT_SETTING_SEG0_3    0x00000000
#define INIT_SETTING_SEG0_4    0x00000000
#define INIT_SETTING_SEG0_5    0x00000000
#define INIT_SETTING_SEG0_6    0x00000000
#define INIT_SETTING_SEG0_7    0x00000000
#define INIT_SETTING_SEG1_0    0x00000000
#define INIT_SETTING_SEG1_1    0x00000000
#define INIT_SETTING_SEG1_2    0x00007F40
#define INIT_SETTING_SEG1_3    0x00006C00
#define INIT_SETTING_SEG1_4    0x00007F30
#define INIT_SETTING_SEG1_5    0x00006800
#define INIT_SETTING_SEG1_6    0x00000000
#define INIT_SETTING_SEG1_7    0x00000000

/****************************************************************************
 * Name: mpfs_setup_ddr_segments
 *
 * Description:
 *   Sets up the MPU configurations properly.
 *
 * Input Parameters:
 *   option  - Default setup, or values from the Libero tool.
 *
 ****************************************************************************/

void mpfs_setup_ddr_segments(enum seg_setup_e option)
{
  uint32_t val_l;
  uint32_t val_h;

  if (option == DEFAULT_SEG_SETUP)
    {
      /* Segment 0 */

      val_l = INIT_SETTING_SEG0_0 & 0x7fff;
      val_h = INIT_SETTING_SEG0_1 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG0_REG0);

      /* Segment 1 */

      val_l = INIT_SETTING_SEG1_2 & 0x7fff;
      val_h = INIT_SETTING_SEG1_3 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG1_REG1);

      val_l = INIT_SETTING_SEG1_4 & 0x7fff;
      val_h = INIT_SETTING_SEG1_5 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG1_REG2);
    }
  else
    {
      /* Segment 0 */

      val_l = LIBERO_SETTING_SEG0_0 & 0x7fff;
      val_h = LIBERO_SETTING_SEG0_1 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG0_REG0);

      /* Segment 1 */

      val_l = LIBERO_SETTING_SEG1_2 & 0x7fff;
      val_h = LIBERO_SETTING_SEG1_3 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG1_REG1);

      val_l = LIBERO_SETTING_SEG1_4 & 0x7fff;
      val_h = LIBERO_SETTING_SEG1_5 & 0x7fff;

      putreg64((((uint64_t)val_h) << 32) | val_l, MPFS_MPUCFG_SEG1_REG2);
    }

  /* Disable ddr blocker: cleared at reset. When written to '1' disables
   * the blocker function allowing the L2 cache controller to access the
   * DDRC. Once written to '1' the register cannot be written to 0, only
   * an devie reset will clear the register.
   */

  putreg64((((uint64_t)0x01) << 32) , MPFS_MPUCFG_SEG0_REG3);
}

/****************************************************************************
 * Name: mpfs_init_ddrc
 *
 * Description:
 *   Assigns value constants provided by the Libero tool and the
 *   manufacturer. This writes good known values into all DDR RW registers.
 *
 ****************************************************************************/

static void mpfs_init_ddrc(void)
{
  putreg32(LIBERO_SETTING_CFG_MANUAL_ADDRESS_MAP,
           MPFS_DDR_CSR_APB_CFG_MANUAL_ADDRESS_MAP);
  putreg32(LIBERO_SETTING_CFG_CHIPADDR_MAP,
           MPFS_DDR_CSR_APB_CFG_CHIPADDR_MAP);
  putreg32(LIBERO_SETTING_CFG_CIDADDR_MAP,
           MPFS_DDR_CSR_APB_CFG_CIDADDR_MAP);
  putreg32(LIBERO_SETTING_CFG_MB_AUTOPCH_COL_BIT_POS_LOW,
           MPFS_DDR_CSR_APB_CFG_MB_AUTOPCH_COL_BIT_POS_LOW);

  putreg32(LIBERO_SETTING_CFG_MB_AUTOPCH_COL_BIT_POS_HIGH,
           MPFS_DDR_CSR_APB_CFG_MB_AUTOPCH_COL_BIT_POS_HIGH);
  putreg32(LIBERO_SETTING_CFG_BANKADDR_MAP_0,
           MPFS_DDR_CSR_APB_CFG_BANKADDR_MAP_0);
  putreg32(LIBERO_SETTING_CFG_BANKADDR_MAP_1,
           MPFS_DDR_CSR_APB_CFG_BANKADDR_MAP_1);
  putreg32(LIBERO_SETTING_CFG_ROWADDR_MAP_0,
           MPFS_DDR_CSR_APB_CFG_ROWADDR_MAP_0);

  putreg32(LIBERO_SETTING_CFG_ROWADDR_MAP_1,
           MPFS_DDR_CSR_APB_CFG_ROWADDR_MAP_1);
  putreg32(LIBERO_SETTING_CFG_ROWADDR_MAP_2,
           MPFS_DDR_CSR_APB_CFG_ROWADDR_MAP_2);
  putreg32(LIBERO_SETTING_CFG_ROWADDR_MAP_3,
           MPFS_DDR_CSR_APB_CFG_ROWADDR_MAP_3);
  putreg32(LIBERO_SETTING_CFG_COLADDR_MAP_0,
           MPFS_DDR_CSR_APB_CFG_COLADDR_MAP_0);

  putreg32(LIBERO_SETTING_CFG_COLADDR_MAP_1,
           MPFS_DDR_CSR_APB_CFG_COLADDR_MAP_1);
  putreg32(LIBERO_SETTING_CFG_COLADDR_MAP_2,
           MPFS_DDR_CSR_APB_CFG_COLADDR_MAP_2);
  putreg32(LIBERO_SETTING_CFG_VRCG_ENABLE,
           MPFS_DDR_CSR_APB_CFG_VRCG_ENABLE);
  putreg32(LIBERO_SETTING_CFG_VRCG_DISABLE,
           MPFS_DDR_CSR_APB_CFG_VRCG_DISABLE);

  putreg32(LIBERO_SETTING_CFG_WRITE_LATENCY_SET,
           MPFS_DDR_CSR_APB_CFG_WRITE_LATENCY_SET);
  putreg32(LIBERO_SETTING_CFG_THERMAL_OFFSET,
           MPFS_DDR_CSR_APB_CFG_THERMAL_OFFSET);
  putreg32(LIBERO_SETTING_CFG_SOC_ODT,
           MPFS_DDR_CSR_APB_CFG_SOC_ODT);
  putreg32(LIBERO_SETTING_CFG_ODTE_CK,
           MPFS_DDR_CSR_APB_CFG_ODTE_CK);

  putreg32(LIBERO_SETTING_CFG_ODTE_CS,
           MPFS_DDR_CSR_APB_CFG_ODTE_CS);
  putreg32(LIBERO_SETTING_CFG_ODTD_CA,
           MPFS_DDR_CSR_APB_CFG_ODTD_CA);
  putreg32(LIBERO_SETTING_CFG_LPDDR4_FSP_OP,
           MPFS_DDR_CSR_APB_CFG_LPDDR4_FSP_OP);
  putreg32(LIBERO_SETTING_CFG_GENERATE_REFRESH_ON_SRX,
           MPFS_DDR_CSR_APB_CFG_GENERATE_REFRESH_ON_SRX);

  putreg32(LIBERO_SETTING_CFG_DBI_CL,
           MPFS_DDR_CSR_APB_CFG_DBI_CL);
  putreg32(LIBERO_SETTING_CFG_NON_DBI_CL,
           MPFS_DDR_CSR_APB_CFG_NON_DBI_CL);
  putreg32(LIBERO_SETTING_INIT_FORCE_WRITE_DATA_0,
           MPFS_DDR_CSR_APB_INIT_FORCE_WRITE_DATA_0);
  putreg32(LIBERO_SETTING_CFG_WRITE_CRC,
           MPFS_DDR_CSR_APB_CFG_WRITE_CRC);

  putreg32(LIBERO_SETTING_CFG_MPR_READ_FORMAT,
           MPFS_DDR_CSR_APB_CFG_MPR_READ_FORMAT);
  putreg32(LIBERO_SETTING_CFG_WR_CMD_LAT_CRC_DM,
           MPFS_DDR_CSR_APB_CFG_WR_CMD_LAT_CRC_DM);
  putreg32(LIBERO_SETTING_CFG_FINE_GRAN_REF_MODE,
           MPFS_DDR_CSR_APB_CFG_FINE_GRAN_REF_MODE);
  putreg32(LIBERO_SETTING_CFG_TEMP_SENSOR_READOUT,
           MPFS_DDR_CSR_APB_CFG_TEMP_SENSOR_READOUT);

  putreg32(LIBERO_SETTING_CFG_PER_DRAM_ADDR_EN,
           MPFS_DDR_CSR_APB_CFG_PER_DRAM_ADDR_EN);
  putreg32(LIBERO_SETTING_CFG_GEARDOWN_MODE,
           MPFS_DDR_CSR_APB_CFG_GEARDOWN_MODE);
  putreg32(LIBERO_SETTING_CFG_WR_PREAMBLE,
           MPFS_DDR_CSR_APB_CFG_WR_PREAMBLE);
  putreg32(LIBERO_SETTING_CFG_RD_PREAMBLE,
           MPFS_DDR_CSR_APB_CFG_RD_PREAMBLE);

  putreg32(LIBERO_SETTING_CFG_RD_PREAMB_TRN_MODE,
           MPFS_DDR_CSR_APB_CFG_RD_PREAMB_TRN_MODE);
  putreg32(LIBERO_SETTING_CFG_SR_ABORT,
           MPFS_DDR_CSR_APB_CFG_SR_ABORT);
  putreg32(LIBERO_SETTING_CFG_CS_TO_CMDADDR_LATENCY,
           MPFS_DDR_CSR_APB_CFG_CS_TO_CMDADDR_LATENCY);
  putreg32(LIBERO_SETTING_CFG_INT_VREF_MON,
           MPFS_DDR_CSR_APB_CFG_INT_VREF_MON);

  putreg32(LIBERO_SETTING_CFG_TEMP_CTRL_REF_MODE,
           MPFS_DDR_CSR_APB_CFG_TEMP_CTRL_REF_MODE);
  putreg32(LIBERO_SETTING_CFG_TEMP_CTRL_REF_RANGE,
           MPFS_DDR_CSR_APB_CFG_TEMP_CTRL_REF_RANGE);
  putreg32(LIBERO_SETTING_CFG_MAX_PWR_DOWN_MODE,
           MPFS_DDR_CSR_APB_CFG_MAX_PWR_DOWN_MODE);
  putreg32(LIBERO_SETTING_CFG_READ_DBI,
           MPFS_DDR_CSR_APB_CFG_READ_DBI);

  putreg32(LIBERO_SETTING_CFG_WRITE_DBI,
           MPFS_DDR_CSR_APB_CFG_WRITE_DBI);
  putreg32(LIBERO_SETTING_CFG_DATA_MASK,
           MPFS_DDR_CSR_APB_CFG_DATA_MASK);
  putreg32(LIBERO_SETTING_CFG_CA_PARITY_PERSIST_ERR,
           MPFS_DDR_CSR_APB_CFG_CA_PARITY_PERSIST_ERR);
  putreg32(LIBERO_SETTING_CFG_RTT_PARK,
           MPFS_DDR_CSR_APB_CFG_RTT_PARK);

  putreg32(LIBERO_SETTING_CFG_ODT_INBUF_4_PD,
           MPFS_DDR_CSR_APB_CFG_ODT_INBUF_4_PD);
  putreg32(LIBERO_SETTING_CFG_CA_PARITY_ERR_STATUS,
           MPFS_DDR_CSR_APB_CFG_CA_PARITY_ERR_STATUS);
  putreg32(LIBERO_SETTING_CFG_CRC_ERROR_CLEAR,
           MPFS_DDR_CSR_APB_CFG_CRC_ERROR_CLEAR);
  putreg32(LIBERO_SETTING_CFG_CA_PARITY_LATENCY,
           MPFS_DDR_CSR_APB_CFG_CA_PARITY_LATENCY);

  putreg32(LIBERO_SETTING_CFG_CCD_S,
           MPFS_DDR_CSR_APB_CFG_CCD_S);
  putreg32(LIBERO_SETTING_CFG_CCD_L,
           MPFS_DDR_CSR_APB_CFG_CCD_L);
  putreg32(LIBERO_SETTING_CFG_VREFDQ_TRN_ENABLE,
           MPFS_DDR_CSR_APB_CFG_VREFDQ_TRN_ENABLE);
  putreg32(LIBERO_SETTING_CFG_VREFDQ_TRN_RANGE,
           MPFS_DDR_CSR_APB_CFG_VREFDQ_TRN_RANGE);

  putreg32(LIBERO_SETTING_CFG_VREFDQ_TRN_VALUE,
           MPFS_DDR_CSR_APB_CFG_VREFDQ_TRN_VALUE);
  putreg32(LIBERO_SETTING_CFG_RRD_S, MPFS_DDR_CSR_APB_CFG_RRD_S);
  putreg32(LIBERO_SETTING_CFG_RRD_L, MPFS_DDR_CSR_APB_CFG_RRD_L);
  putreg32(LIBERO_SETTING_CFG_WTR_S, MPFS_DDR_CSR_APB_CFG_WTR_S);

  putreg32(LIBERO_SETTING_CFG_WTR_L, MPFS_DDR_CSR_APB_CFG_WTR_L);
  putreg32(LIBERO_SETTING_CFG_WTR_S_CRC_DM,
           MPFS_DDR_CSR_APB_CFG_WTR_S_CRC_DM);
  putreg32(LIBERO_SETTING_CFG_WTR_L_CRC_DM,
           MPFS_DDR_CSR_APB_CFG_WTR_L_CRC_DM);
  putreg32(LIBERO_SETTING_CFG_WR_CRC_DM, MPFS_DDR_CSR_APB_CFG_WR_CRC_DM);

  putreg32(LIBERO_SETTING_CFG_RFC1, MPFS_DDR_CSR_APB_CFG_RFC1);
  putreg32(LIBERO_SETTING_CFG_RFC2, MPFS_DDR_CSR_APB_CFG_RFC2);
  putreg32(LIBERO_SETTING_CFG_RFC4, MPFS_DDR_CSR_APB_CFG_RFC4);
  putreg32(LIBERO_SETTING_CFG_NIBBLE_DEVICES,
           MPFS_DDR_CSR_APB_CFG_NIBBLE_DEVICES);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS0_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS0_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS0_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS0_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS1_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS1_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS1_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS1_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS2_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS2_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS2_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS2_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS3_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS3_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS3_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS3_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS4_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS4_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS4_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS4_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS5_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS5_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS5_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS5_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS6_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS6_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS6_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS6_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS7_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS7_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS7_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS7_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS8_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS8_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS8_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS8_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS9_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS9_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS9_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS9_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS10_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS10_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS10_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS10_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS11_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS11_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS11_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS11_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS12_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS12_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS12_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS12_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS13_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS13_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS13_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS13_1);

  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS14_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS14_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS14_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS14_1);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS15_0,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS15_0);
  putreg32(LIBERO_SETTING_CFG_BIT_MAP_INDEX_CS15_1,
           MPFS_DDR_CSR_APB_CFG_BIT_MAP_INDEX_CS15_1);

  putreg32(LIBERO_SETTING_CFG_NUM_LOGICAL_RANKS_PER_3DS,
           MPFS_DDR_CSR_APB_CFG_NUM_LOGICAL_RANKS_PER_3DS);
  putreg32(LIBERO_SETTING_CFG_RFC_DLR1, MPFS_DDR_CSR_APB_CFG_RFC_DLR1);
  putreg32(LIBERO_SETTING_CFG_RFC_DLR2, MPFS_DDR_CSR_APB_CFG_RFC_DLR2);
  putreg32(LIBERO_SETTING_CFG_RFC_DLR4, MPFS_DDR_CSR_APB_CFG_RFC_DLR4);

  putreg32(LIBERO_SETTING_CFG_RRD_DLR, MPFS_DDR_CSR_APB_CFG_RRD_DLR);
  putreg32(LIBERO_SETTING_CFG_FAW_DLR, MPFS_DDR_CSR_APB_CFG_FAW_DLR);
  putreg32(LIBERO_SETTING_CFG_ADVANCE_ACTIVATE_READY,
           MPFS_DDR_CSR_APB_CFG_ADVANCE_ACTIVATE_READY);
  putreg32(LIBERO_SETTING_CTRLR_SOFT_RESET_N,
           MPFS_DDR_CSR_APB_CTRLR_SOFT_RESET_N);

  putreg32(LIBERO_SETTING_CFG_LOOKAHEAD_PCH,
           MPFS_DDR_CSR_APB_CFG_LOOKAHEAD_PCH);
  putreg32(LIBERO_SETTING_CFG_LOOKAHEAD_ACT,
           MPFS_DDR_CSR_APB_CFG_LOOKAHEAD_ACT);
  putreg32(LIBERO_SETTING_INIT_AUTOINIT_DISABLE,
           MPFS_DDR_CSR_APB_INIT_AUTOINIT_DISABLE);
  putreg32(LIBERO_SETTING_INIT_FORCE_RESET,
           MPFS_DDR_CSR_APB_INIT_FORCE_RESET);

  putreg32(LIBERO_SETTING_INIT_GEARDOWN_EN,
           MPFS_DDR_CSR_APB_INIT_GEARDOWN_EN);
  putreg32(LIBERO_SETTING_INIT_DISABLE_CKE,
           MPFS_DDR_CSR_APB_INIT_DISABLE_CKE);
  putreg32(LIBERO_SETTING_INIT_CS,
           MPFS_DDR_CSR_APB_INIT_CS);
  putreg32(LIBERO_SETTING_INIT_PRECHARGE_ALL,
           MPFS_DDR_CSR_APB_INIT_PRECHARGE_ALL);

  putreg32(LIBERO_SETTING_INIT_REFRESH, MPFS_DDR_CSR_APB_INIT_REFRESH);
  putreg32(LIBERO_SETTING_INIT_ZQ_CAL_REQ, MPFS_DDR_CSR_APB_INIT_ZQ_CAL_REQ);
  putreg32(LIBERO_SETTING_CFG_BL, MPFS_DDR_CSR_APB_CFG_BL);
  putreg32(LIBERO_SETTING_CTRLR_INIT, MPFS_DDR_CSR_APB_CTRLR_INIT);

  putreg32(LIBERO_SETTING_CFG_AUTO_REF_EN, MPFS_DDR_CSR_APB_CFG_AUTO_REF_EN);
  putreg32(LIBERO_SETTING_CFG_RAS, MPFS_DDR_CSR_APB_CFG_RAS);
  putreg32(LIBERO_SETTING_CFG_RCD, MPFS_DDR_CSR_APB_CFG_RCD);
  putreg32(LIBERO_SETTING_CFG_RRD, MPFS_DDR_CSR_APB_CFG_RRD);

  putreg32(LIBERO_SETTING_CFG_RP, MPFS_DDR_CSR_APB_CFG_RP);
  putreg32(LIBERO_SETTING_CFG_RC, MPFS_DDR_CSR_APB_CFG_RC);
  putreg32(LIBERO_SETTING_CFG_FAW, MPFS_DDR_CSR_APB_CFG_FAW);
  putreg32(LIBERO_SETTING_CFG_RFC, MPFS_DDR_CSR_APB_CFG_RFC);

  putreg32(LIBERO_SETTING_CFG_RTP, MPFS_DDR_CSR_APB_CFG_RTP);
  putreg32(LIBERO_SETTING_CFG_WR, MPFS_DDR_CSR_APB_CFG_WR);
  putreg32(LIBERO_SETTING_CFG_WTR, MPFS_DDR_CSR_APB_CFG_WTR);
  putreg32(LIBERO_SETTING_CFG_PASR, MPFS_DDR_CSR_APB_CFG_PASR);

  putreg32(LIBERO_SETTING_CFG_XP, MPFS_DDR_CSR_APB_CFG_XP);
  putreg32(LIBERO_SETTING_CFG_XSR, MPFS_DDR_CSR_APB_CFG_XSR);
  putreg32(LIBERO_SETTING_CFG_CL, MPFS_DDR_CSR_APB_CFG_CL);
  putreg32(LIBERO_SETTING_CFG_READ_TO_WRITE,
           MPFS_DDR_CSR_APB_CFG_READ_TO_WRITE);

  putreg32(LIBERO_SETTING_CFG_WRITE_TO_WRITE,
           MPFS_DDR_CSR_APB_CFG_WRITE_TO_WRITE);
  putreg32(LIBERO_SETTING_CFG_WRITE_TO_WRITE,
           MPFS_DDR_CSR_APB_CFG_READ_TO_READ);
  putreg32(LIBERO_SETTING_CFG_WRITE_TO_WRITE,
           MPFS_DDR_CSR_APB_CFG_WRITE_TO_READ);
  putreg32(LIBERO_SETTING_CFG_READ_TO_WRITE_ODT,
           MPFS_DDR_CSR_APB_CFG_READ_TO_WRITE_ODT);

  putreg32(LIBERO_SETTING_CFG_WRITE_TO_WRITE_ODT,
           MPFS_DDR_CSR_APB_CFG_WRITE_TO_WRITE_ODT);
  putreg32(LIBERO_SETTING_CFG_READ_TO_READ_ODT,
           MPFS_DDR_CSR_APB_CFG_READ_TO_READ_ODT);
  putreg32(LIBERO_SETTING_CFG_WRITE_TO_READ_ODT,
           MPFS_DDR_CSR_APB_CFG_WRITE_TO_READ_ODT);
  putreg32(LIBERO_SETTING_CFG_MIN_READ_IDLE,
           MPFS_DDR_CSR_APB_CFG_MIN_READ_IDLE);

  putreg32(LIBERO_SETTING_CFG_MRD, MPFS_DDR_CSR_APB_CFG_MRD);
  putreg32(LIBERO_SETTING_CFG_BT, MPFS_DDR_CSR_APB_CFG_BT);
  putreg32(LIBERO_SETTING_CFG_DS, MPFS_DDR_CSR_APB_CFG_DS);
  putreg32(LIBERO_SETTING_CFG_QOFF, MPFS_DDR_CSR_APB_CFG_QOFF);

  putreg32(LIBERO_SETTING_CFG_RTT, MPFS_DDR_CSR_APB_CFG_RTT);
  putreg32(LIBERO_SETTING_CFG_DLL_DISABLE, MPFS_DDR_CSR_APB_CFG_DLL_DISABLE);
  putreg32(LIBERO_SETTING_CFG_REF_PER, MPFS_DDR_CSR_APB_CFG_REF_PER);
  putreg32(LIBERO_SETTING_CFG_STARTUP_DELAY,
           MPFS_DDR_CSR_APB_CFG_STARTUP_DELAY);

  putreg32(LIBERO_SETTING_CFG_MEM_COLBITS,
           MPFS_DDR_CSR_APB_CFG_MEM_COLBITS);
  putreg32(LIBERO_SETTING_CFG_MEM_ROWBITS,
           MPFS_DDR_CSR_APB_CFG_MEM_ROWBITS);
  putreg32(LIBERO_SETTING_CFG_MEM_BANKBITS,
           MPFS_DDR_CSR_APB_CFG_MEM_BANKBITS);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS0,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS0);

  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS1,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS1);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS2,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS2);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS3,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS3);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS4,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS4);

  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS5,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS5);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS6,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS6);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_MAP_CS7,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_MAP_CS7);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS0,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS0);

  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS1,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS1);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS2,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS2);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS3,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS3);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS4,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS4);

  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS5,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS5);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS6,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS6);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_MAP_CS7,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_MAP_CS7);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_TURN_ON,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_TURN_ON);

  putreg32(LIBERO_SETTING_CFG_ODT_WR_TURN_ON,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_TURN_ON);
  putreg32(LIBERO_SETTING_CFG_ODT_RD_TURN_OFF,
           MPFS_DDR_CSR_APB_CFG_ODT_RD_TURN_OFF);
  putreg32(LIBERO_SETTING_CFG_ODT_WR_TURN_OFF,
           MPFS_DDR_CSR_APB_CFG_ODT_WR_TURN_OFF);
  putreg32(LIBERO_SETTING_CFG_EMR3, MPFS_DDR_CSR_APB_CFG_EMR3);

  putreg32(LIBERO_SETTING_CFG_TWO_T, MPFS_DDR_CSR_APB_CFG_TWO_T);
  putreg32(LIBERO_SETTING_CFG_TWO_T_SEL_CYCLE,
           MPFS_DDR_CSR_APB_CFG_TWO_T_SEL_CYCLE);
  putreg32(LIBERO_SETTING_CFG_REGDIMM, MPFS_DDR_CSR_APB_CFG_REGDIMM);
  putreg32(LIBERO_SETTING_CFG_MOD, MPFS_DDR_CSR_APB_CFG_MOD);

  putreg32(LIBERO_SETTING_CFG_XS, MPFS_DDR_CSR_APB_CFG_XS);
  putreg32(LIBERO_SETTING_CFG_XSDLL, MPFS_DDR_CSR_APB_CFG_XSDLL);
  putreg32(LIBERO_SETTING_CFG_XPR, MPFS_DDR_CSR_APB_CFG_XPR);
  putreg32(LIBERO_SETTING_CFG_AL_MODE, MPFS_DDR_CSR_APB_CFG_AL_MODE);

  putreg32(LIBERO_SETTING_CFG_CWL, MPFS_DDR_CSR_APB_CFG_CWL);
  putreg32(LIBERO_SETTING_CFG_BL_MODE, MPFS_DDR_CSR_APB_CFG_BL_MODE);
  putreg32(LIBERO_SETTING_CFG_TDQS, MPFS_DDR_CSR_APB_CFG_TDQS);
  putreg32(LIBERO_SETTING_CFG_RTT_WR, MPFS_DDR_CSR_APB_CFG_RTT_WR);

  putreg32(LIBERO_SETTING_CFG_LP_ASR, MPFS_DDR_CSR_APB_CFG_LP_ASR);
  putreg32(LIBERO_SETTING_CFG_AUTO_SR, MPFS_DDR_CSR_APB_CFG_AUTO_SR);
  putreg32(LIBERO_SETTING_CFG_SRT, MPFS_DDR_CSR_APB_CFG_SRT);
  putreg32(LIBERO_SETTING_CFG_ADDR_MIRROR, MPFS_DDR_CSR_APB_CFG_ADDR_MIRROR);

  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_TYPE, MPFS_DDR_CSR_APB_CFG_ZQ_CAL_TYPE);
  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_PER, MPFS_DDR_CSR_APB_CFG_ZQ_CAL_PER);
  putreg32(LIBERO_SETTING_CFG_AUTO_ZQ_CAL_EN,
           MPFS_DDR_CSR_APB_CFG_AUTO_ZQ_CAL_EN);
  putreg32(LIBERO_SETTING_CFG_MEMORY_TYPE, MPFS_DDR_CSR_APB_CFG_MEMORY_TYPE);

  putreg32(LIBERO_SETTING_CFG_ONLY_SRANK_CMDS,
           MPFS_DDR_CSR_APB_CFG_ONLY_SRANK_CMDS);
  putreg32(LIBERO_SETTING_CFG_NUM_RANKS, MPFS_DDR_CSR_APB_CFG_NUM_RANKS);
  putreg32(LIBERO_SETTING_CFG_QUAD_RANK, MPFS_DDR_CSR_APB_CFG_QUAD_RANK);
  putreg32(LIBERO_SETTING_CFG_EARLY_RANK_TO_WR_START,
           MPFS_DDR_CSR_APB_CFG_EARLY_RANK_TO_WR_START);

  putreg32(LIBERO_SETTING_CFG_EARLY_RANK_TO_RD_START,
           MPFS_DDR_CSR_APB_CFG_EARLY_RANK_TO_RD_START);
  putreg32(LIBERO_SETTING_CFG_PASR_BANK, MPFS_DDR_CSR_APB_CFG_PASR_BANK);
  putreg32(LIBERO_SETTING_CFG_PASR_SEG, MPFS_DDR_CSR_APB_CFG_PASR_SEG);
  putreg32(LIBERO_SETTING_INIT_MRR_MODE, MPFS_DDR_CSR_APB_INIT_MRR_MODE);

  putreg32(LIBERO_SETTING_INIT_MR_W_REQ, MPFS_DDR_CSR_APB_INIT_MR_W_REQ);
  putreg32(LIBERO_SETTING_INIT_MR_ADDR, MPFS_DDR_CSR_APB_INIT_MR_ADDR);
  putreg32(LIBERO_SETTING_INIT_MR_WR_DATA, MPFS_DDR_CSR_APB_INIT_MR_WR_DATA);
  putreg32(LIBERO_SETTING_INIT_MR_WR_MASK, MPFS_DDR_CSR_APB_INIT_MR_WR_MASK);

  putreg32(LIBERO_SETTING_INIT_NOP, MPFS_DDR_CSR_APB_INIT_NOP);
  putreg32(LIBERO_SETTING_CFG_INIT_DURATION,
           MPFS_DDR_CSR_APB_CFG_INIT_DURATION);
  putreg32(LIBERO_SETTING_CFG_ZQINIT_CAL_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQINIT_CAL_DURATION);
  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_L_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQ_CAL_L_DURATION);

  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_S_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQ_CAL_S_DURATION);
  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_R_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQ_CAL_R_DURATION);
  putreg32(LIBERO_SETTING_CFG_MRR, MPFS_DDR_CSR_APB_CFG_MRR);
  putreg32(LIBERO_SETTING_CFG_MRW, MPFS_DDR_CSR_APB_CFG_MRW);

  putreg32(LIBERO_SETTING_CFG_ODT_POWERDOWN,
           MPFS_DDR_CSR_APB_CFG_ODT_POWERDOWN);
  putreg32(LIBERO_SETTING_CFG_WL, MPFS_DDR_CSR_APB_CFG_WL);
  putreg32(LIBERO_SETTING_CFG_RL, MPFS_DDR_CSR_APB_CFG_RL);
  putreg32(LIBERO_SETTING_CFG_CAL_READ_PERIOD,
           MPFS_DDR_CSR_APB_CFG_CAL_READ_PERIOD);

  putreg32(LIBERO_SETTING_CFG_NUM_CAL_READS,
           MPFS_DDR_CSR_APB_CFG_NUM_CAL_READS);
  putreg32(LIBERO_SETTING_INIT_SELF_REFRESH,
           MPFS_DDR_CSR_APB_INIT_SELF_REFRESH);
  putreg32(LIBERO_SETTING_INIT_POWER_DOWN,
           MPFS_DDR_CSR_APB_INIT_POWER_DOWN);
  putreg32(LIBERO_SETTING_INIT_FORCE_WRITE,
           MPFS_DDR_CSR_APB_INIT_FORCE_WRITE);

  putreg32(LIBERO_SETTING_INIT_FORCE_WRITE_CS,
           MPFS_DDR_CSR_APB_INIT_FORCE_WRITE_CS);
  putreg32(LIBERO_SETTING_CFG_CTRLR_INIT_DISABLE,
           MPFS_DDR_CSR_APB_CFG_CTRLR_INIT_DISABLE);
  putreg32(LIBERO_SETTING_INIT_RDIMM_COMPLETE,
           MPFS_DDR_CSR_APB_INIT_RDIMM_COMPLETE);
  putreg32(LIBERO_SETTING_CFG_RDIMM_LAT,
           MPFS_DDR_CSR_APB_CFG_RDIMM_LAT);

  putreg32(LIBERO_SETTING_CFG_RDIMM_BSIDE_INVERT,
           MPFS_DDR_CSR_APB_CFG_RDIMM_BSIDE_INVERT);
  putreg32(LIBERO_SETTING_CFG_LRDIMM,
           MPFS_DDR_CSR_APB_CFG_LRDIMM);
  putreg32(LIBERO_SETTING_INIT_MEMORY_RESET_MASK,
           MPFS_DDR_CSR_APB_INIT_MEMORY_RESET_MASK);
  putreg32(LIBERO_SETTING_CFG_RD_PREAMB_TOGGLE,
           MPFS_DDR_CSR_APB_CFG_RD_PREAMB_TOGGLE);

  putreg32(LIBERO_SETTING_CFG_RD_POSTAMBLE,
           MPFS_DDR_CSR_APB_CFG_RD_POSTAMBLE);
  putreg32(LIBERO_SETTING_CFG_PU_CAL, MPFS_DDR_CSR_APB_CFG_PU_CAL);
  putreg32(LIBERO_SETTING_CFG_DQ_ODT, MPFS_DDR_CSR_APB_CFG_DQ_ODT);
  putreg32(LIBERO_SETTING_CFG_CA_ODT, MPFS_DDR_CSR_APB_CFG_CA_ODT);

  putreg32(LIBERO_SETTING_CFG_ZQLATCH_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQLATCH_DURATION);
  putreg32(LIBERO_SETTING_INIT_CAL_SELECT,
           MPFS_DDR_CSR_APB_INIT_CAL_SELECT);
  putreg32(LIBERO_SETTING_INIT_CAL_L_R_REQ,
           MPFS_DDR_CSR_APB_INIT_CAL_L_R_REQ);
  putreg32(LIBERO_SETTING_INIT_CAL_L_B_SIZE,
           MPFS_DDR_CSR_APB_INIT_CAL_L_B_SIZE);

  putreg32(LIBERO_SETTING_INIT_RWFIFO, MPFS_DDR_CSR_APB_INIT_RWFIFO);
  putreg32(LIBERO_SETTING_INIT_RD_DQCAL, MPFS_DDR_CSR_APB_INIT_RD_DQCAL);
  putreg32(LIBERO_SETTING_INIT_START_DQSOSC,
           MPFS_DDR_CSR_APB_INIT_START_DQSOSC);
  putreg32(LIBERO_SETTING_INIT_STOP_DQSOSC,
           MPFS_DDR_CSR_APB_INIT_STOP_DQSOSC);

  putreg32(LIBERO_SETTING_INIT_ZQ_CAL_START,
           MPFS_DDR_CSR_APB_INIT_ZQ_CAL_START);
  putreg32(LIBERO_SETTING_CFG_WR_POSTAMBLE,
           MPFS_DDR_CSR_APB_CFG_WR_POSTAMBLE);
  putreg32(LIBERO_SETTING_INIT_CAL_L_ADDR_0,
           MPFS_DDR_CSR_APB_INIT_CAL_L_ADDR_0);
  putreg32(LIBERO_SETTING_INIT_CAL_L_ADDR_1,
           MPFS_DDR_CSR_APB_INIT_CAL_L_ADDR_1);

  putreg32(LIBERO_SETTING_CFG_CTRLUPD_TRIG,
           MPFS_DDR_CSR_APB_CFG_CTRLUPD_TRIG);
  putreg32(LIBERO_SETTING_CFG_CTRLUPD_START_DELAY,
           MPFS_DDR_CSR_APB_CFG_CTRLUPD_START_DELAY);
  putreg32(LIBERO_SETTING_CFG_DFI_T_CTRLUPD_MAX,
           MPFS_DDR_CSR_APB_CFG_DFI_T_CTRLUPD_MAX);
  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_SEL,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_SEL);

  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_VALUE,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_VALUE);
  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_TURN_OFF_DELAY,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_TURN_OFF_DELAY);
  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_SLOW_RESTART_WIN,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_SLOW_RESTART_WINDOW);
  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_RESTART_HOLDOFF,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_RESTART_HOLDOFF);

  putreg32(LIBERO_SETTING_CFG_PARITY_RDIMM_DELAY,
           MPFS_DDR_CSR_APB_CFG_PARITY_RDIMM_DELAY);
  putreg32(LIBERO_SETTING_CFG_CTRLR_BUSY_ENABLE,
           MPFS_DDR_CSR_APB_CFG_CTRLR_BUSY_ENABLE);
  putreg32(LIBERO_SETTING_CFG_ASYNC_ODT,
           MPFS_DDR_CSR_APB_CFG_ASYNC_ODT);
  putreg32(LIBERO_SETTING_CFG_ZQ_CAL_DURATION,
           MPFS_DDR_CSR_APB_CFG_ZQ_CAL_DURATION);

  putreg32(LIBERO_SETTING_CFG_MRRI, MPFS_DDR_CSR_APB_CFG_MRRI);
  putreg32(LIBERO_SETTING_INIT_ODT_FORCE_EN,
           MPFS_DDR_CSR_APB_INIT_ODT_FORCE_EN);
  putreg32(LIBERO_SETTING_INIT_ODT_FORCE_RANK,
           MPFS_DDR_CSR_APB_INIT_ODT_FORCE_RANK);
  putreg32(LIBERO_SETTING_CFG_PHYUPD_ACK_DELAY,
           MPFS_DDR_CSR_APB_CFG_PHYUPD_ACK_DELAY);

  putreg32(LIBERO_SETTING_CFG_MIRROR_X16_BG0_BG1,
           MPFS_DDR_CSR_APB_CFG_MIRROR_X16_BG0_BG1);
  putreg32(LIBERO_SETTING_INIT_PDA_MR_W_REQ,
           MPFS_DDR_CSR_APB_INIT_PDA_MR_W_REQ);
  putreg32(LIBERO_SETTING_INIT_PDA_NIBBLE_SELECT,
           MPFS_DDR_CSR_APB_INIT_PDA_NIBBLE_SELECT);
  putreg32(LIBERO_SETTING_CFG_DRAM_CLK_DISABLE_IN_SELF_RFH,
           MPFS_DDR_CSR_APB_CFG_DRAM_CLK_DISABLE_IN_SELF_REFRESH);

  putreg32(LIBERO_SETTING_CFG_CKSRE, MPFS_DDR_CSR_APB_CFG_CKSRE);
  putreg32(LIBERO_SETTING_CFG_CKSRX, MPFS_DDR_CSR_APB_CFG_CKSRX);
  putreg32(LIBERO_SETTING_CFG_RCD_STAB, MPFS_DDR_CSR_APB_CFG_RCD_STAB);
  putreg32(LIBERO_SETTING_CFG_DFI_T_CTRL_DELAY,
           MPFS_DDR_CSR_APB_CFG_DFI_T_CTRL_DELAY);

  putreg32(LIBERO_SETTING_CFG_DFI_T_DRAM_CLK_ENABLE,
           MPFS_DDR_CSR_APB_CFG_DFI_T_DRAM_CLK_ENABLE);
  putreg32(LIBERO_SETTING_CFG_IDLE_TIME_TO_SELF_REFRESH,
           MPFS_DDR_CSR_APB_CFG_IDLE_TIME_TO_SELF_REFRESH);
  putreg32(LIBERO_SETTING_CFG_IDLE_TIME_TO_POWER_DOWN,
           MPFS_DDR_CSR_APB_CFG_IDLE_TIME_TO_POWER_DOWN);
  putreg32(LIBERO_SETTING_CFG_BURST_RW_REFRESH_HOLDOFF,
           MPFS_DDR_CSR_APB_CFG_BURST_RW_REFRESH_HOLDOFF);

  putreg32(LIBERO_SETTING_CFG_BG_INTERLEAVE,
           MPFS_DDR_CSR_APB_CFG_BG_INTERLEAVE);
  putreg32(LIBERO_SETTING_CFG_REFRESH_DURING_PHY_TRAINING,
           MPFS_DDR_CSR_APB_CFG_REFRESH_DURING_PHY_TRAINING);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P0,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P0);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P1,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P1);

  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P2,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P2);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P3,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P3);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P4,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P4);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P5,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P5);

  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P6,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P6);
  putreg32(LIBERO_SETTING_CFG_STARVE_TIMEOUT_P7,
           MPFS_DDR_CSR_APB_CFG_STARVE_TIMEOUT_P7);
  putreg32(LIBERO_SETTING_CFG_REORDER_EN,
           MPFS_DDR_CSR_APB_CFG_REORDER_EN);
  putreg32(LIBERO_SETTING_CFG_REORDER_QUEUE_EN,
           MPFS_DDR_CSR_APB_CFG_REORDER_QUEUE_EN);

  putreg32(LIBERO_SETTING_CFG_INTRAPORT_REORDER_EN,
           MPFS_DDR_CSR_APB_CFG_INTRAPORT_REORDER_EN);
  putreg32(LIBERO_SETTING_CFG_INTRAPORT_REORDER_EN,
           MPFS_DDR_CSR_APB_CFG_MAINTAIN_COHERENCY);
  putreg32(LIBERO_SETTING_CFG_Q_AGE_LIMIT,
           MPFS_DDR_CSR_APB_CFG_Q_AGE_LIMIT);
  putreg32(LIBERO_SETTING_CFG_RO_CLOSED_PAGE_POLICY,
           MPFS_DDR_CSR_APB_CFG_RO_CLOSED_PAGE_POLICY);

  putreg32(LIBERO_SETTING_CFG_REORDER_RW_ONLY,
           MPFS_DDR_CSR_APB_CFG_REORDER_RW_ONLY);
  putreg32(LIBERO_SETTING_CFG_RO_PRIORITY_EN,
           MPFS_DDR_CSR_APB_CFG_RO_PRIORITY_EN);
  putreg32(LIBERO_SETTING_CFG_DM_EN, MPFS_DDR_CSR_APB_CFG_DM_EN);
  putreg32(LIBERO_SETTING_CFG_RMW_EN, MPFS_DDR_CSR_APB_CFG_RMW_EN);

  putreg32(LIBERO_SETTING_CFG_ECC_CORRECTION_EN,
           MPFS_DDR_CSR_APB_CFG_ECC_CORRECTION_EN);
  putreg32(LIBERO_SETTING_CFG_ECC_BYPASS,
           MPFS_DDR_CSR_APB_CFG_ECC_BYPASS);
  putreg32(LIBERO_SETTING_INIT_WRITE_DATA_1B_ECC_ERROR_GEN,
           MPFS_DDR_CSR_APB_INIT_WRITE_DATA_1B_ECC_ERROR_GEN);
  putreg32(LIBERO_SETTING_INIT_WRITE_DATA_2B_ECC_ERROR_GEN,
           MPFS_DDR_CSR_APB_INIT_WRITE_DATA_2B_ECC_ERROR_GEN);

  putreg32(LIBERO_SETTING_CFG_ECC_1BIT_INT_THRESH,
           MPFS_DDR_CSR_APB_CFG_ECC_1BIT_INT_THRESH);
  putreg32(LIBERO_SETTING_INIT_READ_CAPTURE_ADDR,
           MPFS_DDR_CSR_APB_INIT_READ_CAPTURE_ADDR);
  putreg32(LIBERO_SETTING_CFG_ERROR_GROUP_SEL,
           MPFS_DDR_CSR_APB_CFG_ERROR_GROUP_SEL);
  putreg32(LIBERO_SETTING_CFG_DATA_SEL,
           MPFS_DDR_CSR_APB_CFG_DATA_SEL);

  putreg32(LIBERO_SETTING_CFG_TRIG_MODE,
           MPFS_DDR_CSR_APB_CFG_TRIG_MODE);
  putreg32(LIBERO_SETTING_CFG_POST_TRIG_CYCS,
           MPFS_DDR_CSR_APB_CFG_POST_TRIG_CYCS);
  putreg32(LIBERO_SETTING_CFG_TRIG_MASK,
           MPFS_DDR_CSR_APB_CFG_TRIG_MASK);
  putreg32(LIBERO_SETTING_CFG_EN_MASK,
           MPFS_DDR_CSR_APB_CFG_EN_MASK);

  putreg32(LIBERO_SETTING_MTC_ACQ_ADDR, MPFS_DDR_CSR_APB_MTC_ACQ_ADDR);
  putreg32(LIBERO_SETTING_CFG_TRIG_MT_ADDR_0,
           MPFS_DDR_CSR_APB_CFG_TRIG_MT_ADDR_0);
  putreg32(LIBERO_SETTING_CFG_TRIG_MT_ADDR_1,
           MPFS_DDR_CSR_APB_CFG_TRIG_MT_ADDR_1);
  putreg32(LIBERO_SETTING_CFG_TRIG_ERR_MASK_0,
           MPFS_DDR_CSR_APB_MT_ERROR_MASK_0);

  putreg32(LIBERO_SETTING_CFG_TRIG_ERR_MASK_1,
           MPFS_DDR_CSR_APB_MT_ERROR_MASK_1);
  putreg32(LIBERO_SETTING_CFG_TRIG_ERR_MASK_2,
           MPFS_DDR_CSR_APB_MT_ERROR_MASK_2);
  putreg32(LIBERO_SETTING_CFG_TRIG_ERR_MASK_3,
           MPFS_DDR_CSR_APB_MT_ERROR_MASK_3);
  putreg32(LIBERO_SETTING_CFG_TRIG_ERR_MASK_4,
           MPFS_DDR_CSR_APB_MT_ERROR_MASK_4);

  putreg32(LIBERO_SETTING_MTC_ACQ_WR_DATA_0,
           MPFS_DDR_CSR_APB_MTC_ACQ_WR_DATA_0);
  putreg32(LIBERO_SETTING_MTC_ACQ_WR_DATA_1,
           MPFS_DDR_CSR_APB_MTC_ACQ_WR_DATA_1);
  putreg32(LIBERO_SETTING_MTC_ACQ_WR_DATA_2,
           MPFS_DDR_CSR_APB_MTC_ACQ_WR_DATA_2);
  putreg32(LIBERO_SETTING_CFG_PRE_TRIG_CYCS,
           MPFS_DDR_CSR_APB_CFG_PRE_TRIG_CYCS);

  putreg32(LIBERO_SETTING_CFG_DATA_SEL_FIRST_ERROR,
           MPFS_DDR_CSR_APB_CFG_DATA_SEL_FIRST_ERROR);
  putreg32(LIBERO_SETTING_CFG_DQ_WIDTH, MPFS_DDR_CSR_APB_CFG_DQ_WIDTH);
  putreg32(LIBERO_SETTING_CFG_ACTIVE_DQ_SEL,
           MPFS_DDR_CSR_APB_CFG_ACTIVE_DQ_SEL);
  putreg32(LIBERO_SETTING_INIT_CA_PARITY_ERROR_GEN_REQ,
           MPFS_DDR_CSR_APB_INIT_CA_PARITY_ERROR_GEN_REQ);
  putreg32(LIBERO_SETTING_INIT_CA_PARITY_ERROR_GEN_CMD,
           MPFS_DDR_CSR_APB_INIT_CA_PARITY_ERROR_GEN_CMD);

  putreg32(LIBERO_SETTING_CFG_DFI_T_RDDATA_EN,
           MPFS_DDR_CSR_APB_CFG_DFI_T_RDDATA_EN);
  putreg32(LIBERO_SETTING_CFG_DFI_T_PHY_RDLAT,
           MPFS_DDR_CSR_APB_CFG_DFI_T_PHY_RDLAT);
  putreg32(LIBERO_SETTING_CFG_DFI_T_PHY_WRLAT,
           MPFS_DDR_CSR_APB_CFG_DFI_T_PHY_WRLAT);
  putreg32(LIBERO_SETTING_CFG_DFI_PHYUPD_EN,
           MPFS_DDR_CSR_APB_CFG_DFI_PHYUPD_EN);

  putreg32(LIBERO_SETTING_INIT_DFI_LP_DATA_REQ,
           MPFS_DDR_CSR_APB_INIT_DFI_LP_DATA_REQ);
  putreg32(LIBERO_SETTING_INIT_DFI_LP_CTRL_REQ,
           MPFS_DDR_CSR_APB_INIT_DFI_LP_CTRL_REQ);
  putreg32(LIBERO_SETTING_INIT_DFI_LP_WAKEUP,
           MPFS_DDR_CSR_APB_INIT_DFI_LP_WAKEUP);
  putreg32(LIBERO_SETTING_INIT_DFI_DRAM_CLK_DISABLE,
           MPFS_DDR_CSR_APB_INIT_DFI_DRAM_CLK_DISABLE);

  putreg32(LIBERO_SETTING_CFG_DFI_DATA_BYTE_DISABLE,
           MPFS_DDR_CSR_APB_CFG_DFI_DATA_BYTE_DISABLE);
  putreg32(LIBERO_SETTING_CFG_DFI_LVL_SEL,
           MPFS_DDR_CSR_APB_CFG_DFI_LVL_SEL);
  putreg32(LIBERO_SETTING_CFG_DFI_LVL_PERIODIC,
           MPFS_DDR_CSR_APB_CFG_DFI_LVL_PERIODIC);
  putreg32(LIBERO_SETTING_CFG_DFI_LVL_PATTERN,
           MPFS_DDR_CSR_APB_CFG_DFI_LVL_PATTERN);

  putreg32(LIBERO_SETTING_PHY_DFI_INIT_START,
           MPFS_DDR_CSR_APB_PHY_DFI_INIT_START);
  putreg32(LIBERO_SETTING_CFG_AXI_START_ADDRESS_AXI1_0,
           MPFS_DDR_CSR_APB_CFG_AXI_START_ADDRESS_AXI1_0);
  putreg32(LIBERO_SETTING_CFG_AXI_START_ADDRESS_AXI1_1,
           MPFS_DDR_CSR_APB_CFG_AXI_START_ADDRESS_AXI1_1);
  putreg32(LIBERO_SETTING_CFG_AXI_START_ADDRESS_AXI2_0,
           MPFS_DDR_CSR_APB_CFG_AXI_START_ADDRESS_AXI2_0);

  putreg32(LIBERO_SETTING_CFG_AXI_START_ADDRESS_AXI2_1,
           MPFS_DDR_CSR_APB_CFG_AXI_START_ADDRESS_AXI2_1);
  putreg32(LIBERO_SETTING_CFG_AXI_END_ADDRESS_AXI1_0,
           MPFS_DDR_CSR_APB_CFG_AXI_END_ADDRESS_AXI1_0);
  putreg32(LIBERO_SETTING_CFG_AXI_END_ADDRESS_AXI1_1,
           MPFS_DDR_CSR_APB_CFG_AXI_END_ADDRESS_AXI1_1);
  putreg32(LIBERO_SETTING_CFG_AXI_END_ADDRESS_AXI2_0,
           MPFS_DDR_CSR_APB_CFG_AXI_END_ADDRESS_AXI2_0);

  putreg32(LIBERO_SETTING_CFG_AXI_END_ADDRESS_AXI2_1,
           MPFS_DDR_CSR_APB_CFG_AXI_END_ADDRESS_AXI2_1);
  putreg32(LIBERO_SETTING_CFG_MEM_START_ADDRESS_AXI1_0,
           MPFS_DDR_CSR_APB_CFG_MEM_START_ADDRESS_AXI1_0);
  putreg32(LIBERO_SETTING_CFG_MEM_START_ADDRESS_AXI1_1,
           MPFS_DDR_CSR_APB_CFG_MEM_START_ADDRESS_AXI1_1);
  putreg32(LIBERO_SETTING_CFG_MEM_START_ADDRESS_AXI2_0,
           MPFS_DDR_CSR_APB_CFG_MEM_START_ADDRESS_AXI2_0);

  putreg32(LIBERO_SETTING_CFG_MEM_START_ADDRESS_AXI2_1,
           MPFS_DDR_CSR_APB_CFG_MEM_START_ADDRESS_AXI2_1);
  putreg32(LIBERO_SETTING_CFG_ENABLE_BUS_HOLD_AXI1,
           MPFS_DDR_CSR_APB_CFG_ENABLE_BUS_HOLD_AXI1);
  putreg32(LIBERO_SETTING_CFG_ENABLE_BUS_HOLD_AXI2,
           MPFS_DDR_CSR_APB_CFG_ENABLE_BUS_HOLD_AXI2);
  putreg32(LIBERO_SETTING_CFG_AXI_AUTO_PCH,
           MPFS_DDR_CSR_APB_CFG_AXI_AUTO_PCH);

  putreg32(LIBERO_SETTING_PHY_RESET_CONTROL,
           MPFS_DDR_CSR_APB_PHY_RESET_CONTROL);
  modifyreg32(MPFS_DDR_CSR_APB_PHY_RESET_CONTROL, 0x8000, 0);
  putreg32(LIBERO_SETTING_PHY_PC_RANK, MPFS_DDR_CSR_APB_PHY_PC_RANK);
  putreg32(LIBERO_SETTING_PHY_RANKS_TO_TRAIN,
           MPFS_DDR_CSR_APB_PHY_RANKS_TO_TRAIN);

  putreg32(LIBERO_SETTING_PHY_WRITE_REQUEST,
           MPFS_DDR_CSR_APB_PHY_WRITE_REQUEST);
  putreg32(LIBERO_SETTING_PHY_READ_REQUEST,
           MPFS_DDR_CSR_APB_PHY_READ_REQUEST);
  putreg32(LIBERO_SETTING_PHY_WRITE_LEVEL_DELAY,
           MPFS_DDR_CSR_APB_PHY_WRITE_LEVEL_DELAY);
  putreg32(LIBERO_SETTING_PHY_GATE_TRAIN_DELAY,
           MPFS_DDR_CSR_APB_PHY_GATE_TRAIN_DELAY);

  putreg32(LIBERO_SETTING_PHY_EYE_TRAIN_DELAY,
           MPFS_DDR_CSR_APB_PHY_EYE_TRAIN_DELAY);
  putreg32(LIBERO_SETTING_PHY_EYE_PAT,
           MPFS_DDR_CSR_APB_PHY_EYE_PAT);
  putreg32(LIBERO_SETTING_PHY_START_RECAL,
           MPFS_DDR_CSR_APB_PHY_START_RECAL);
  putreg32(LIBERO_SETTING_PHY_CLR_DFI_LVL_PERIODIC,
           MPFS_DDR_CSR_APB_PHY_CLR_DFI_LVL_PERIODIC);

  putreg32(LIBERO_SETTING_PHY_TRAIN_STEP_ENABLE,
           MPFS_DDR_CSR_APB_PHY_TRAIN_STEP_ENABLE);
  putreg32(LIBERO_SETTING_PHY_LPDDR_DQ_CAL_PAT,
           MPFS_DDR_CSR_APB_PHY_LPDDR_DQ_CAL_PAT);
  putreg32(LIBERO_SETTING_PHY_INDPNDT_TRAINING,
           MPFS_DDR_CSR_APB_PHY_INDPNDT_TRAINING);
  putreg32(LIBERO_SETTING_PHY_ENCODED_QUAD_CS,
           MPFS_DDR_CSR_APB_PHY_ENCODED_QUAD_CS);

  putreg32(LIBERO_SETTING_PHY_HALF_CLK_DLY_ENABLE,
           MPFS_DDR_CSR_APB_PHY_HALF_CLK_DLY_ENABLE);
}

/****************************************************************************
 * Name: mpfs_ddr_manual_addcmd_refclk_offset
 *
 * Description:
 *   This function determines current sweep offset based on DDR type.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Sweep offset
 *
 ****************************************************************************/

static uint8_t mpfs_ddr_manual_addcmd_refclk_offset(
  struct mpfs_ddr_priv_s *priv)
{
  uint8_t refclk_offset;
  uint8_t type_array_index;

  type_array_index = (uint8_t)priv->ddr_type;
  switch (priv->ddr_type)
    {
      case DDR3L:
      case DDR3:
        if (LIBERO_SETTING_DDR_CLK + DDR_FREQ_MARGIN < DDR_1333_MHZ)
          {
            type_array_index = type_array_index + (uint8_t)LPDDR4 + 1;
          }
          break;

        case DDR4:
        case LPDDR3:
        case LPDDR4:
          if (LIBERO_SETTING_DDR_CLK + DDR_FREQ_MARGIN < DDR_1600_MHZ)
            {
              type_array_index = type_array_index + (uint8_t)LPDDR4 + 1;
            }
          break;

        default:
        case DDR_OFF_MODE:
          break;
    }

  DEBUGASSERT(type_array_index < sizeof(refclk_offsets) /
              sizeof(refclk_offsets[0]));

  if (priv->refclk_sweep_index >= refclk_offsets[type_array_index][0])
    {
      priv->refclk_sweep_index = 0;
    }

  refclk_offset = refclk_offsets[type_array_index] \
                  [priv->refclk_sweep_index + 1];

  priv->refclk_sweep_index = (priv->refclk_sweep_index + 1);

  return refclk_offset;
}

/****************************************************************************
 * Name: mpfs_get_num_lanes
 *
 * Description:
 *   This function returns the number of lanes present.
 *
 * Returned Value:
 *   Number of lanes
 *
 ****************************************************************************/

static uint8_t mpfs_get_num_lanes(void)
{
  uint8_t lanes;

  /* Check width, 16-bit or 32-bit supported, 1 => 32-bit */

  if ((LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_BUS_WIDTH_MASK) ==
            DDRPHY_MODE_BUS_WIDTH_4_LANE)
    {
      lanes = 4;
    }
  else
    {
      lanes = 2;
    }

  /* Check if using ECC, add a lane */

  if ((LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_ECC_MASK) ==
            DDRPHY_MODE_ECC_ON)
    {
      lanes++;
    }

  return lanes;
}

/****************************************************************************
 * Name: mpfs_load_dq
 *
 * Description:
 *   This function loads the DQ for the corresponding lane.
 *
 * Input Parameters:
 *   lane   - the lane number
 *
 ****************************************************************************/

static void mpfs_load_dq(uint8_t lane)
{
  if (lane < 4)
    {
      putreg32(0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG0);
    }
  else
    {
      modifyreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1, 0x0f, 0);
    }

  /* Set expert_dfi_status_override_to_shim = 0x7 */

  putreg32(0x07, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE);

  putreg32(0x21, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  if (lane < 4)
    {
      putreg32(0xffu << (lane * 8),
               MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
    }
  else
    {
      modifyreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1, 0, 0x0f);
    }

  putreg32(0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);

  if (lane < 4)
    {
      putreg32(0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
    }
  else
    {
      modifyreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1, 0x0f, 0);
    }

  putreg32(0x08, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);
}

/****************************************************************************
 * Name: mpfs_mtc_test
 *
 * Description:
 *   This performs a memory test with the NWL memory test core.
 *
 * Input Parameters:
 *   mask             - Test bitmask
 *   start_address    - Test start address
 *   data_pattern     - Data pattern for testing
 *   add_pattern      - Data modifier pattern
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a fail.
 *
 ****************************************************************************/

static uint8_t mpfs_mtc_test(uint8_t mask, uint64_t start_address,
                             uint32_t size,
                             enum mtc_pattern_e data_pattern,
                             enum mtc_add_pattern_e add_pattern)
{
  /* Write calibration:
   *  Configure common memory test interface by writing registers:
   *  MT_STOP_ON_ERROR, MT_DATA_PATTERN, MT_ADDR_PATTERN, MT_ADDR_BITS
   */

  putreg32(0, MPFS_DDR_CSR_APB_MT_STOP_ON_ERROR);

  /* make sure off, will turn on later. */

  putreg32(0, MPFS_DDR_CSR_APB_MT_EN_SINGLE);

  /* MT_DATA_PATTERN:
   *
   * 0x00 => Counting pattern
   * 0x01 => walking 1's
   * 0x02 => pseudo mpfs_ddr_random
   * 0x03 => no repeating pseudo mpfs_ddr_random
   * 0x04 => alt 1's and 0's
   * 0x05 => alt 5's and A's
   * 0x06 => User specified
   * 0x07 => pseudo mpfs_ddr_random 16-bit
   * 0x08 => pseudo mpfs_ddr_random 8-bit
   * 0x09- 0x0f reserved
   *
   */

  /* Added changing pattern so write pattern is different, read back
   * cannot pass on previously written data.
   */

  putreg32(data_pattern, MPFS_DDR_CSR_APB_MT_DATA_PATTERN);

  if (add_pattern == MTC_ADD_RANDOM)
    {
      /* MT_ADDR_PATTERN
       * 0x00 => Count in pattern
       * 0x01 => Pseudo Random Pattern
       * 0x02 => Arbiatry Pattern Gen (user defined ) - Using RAMS
       */

      putreg32(1, MPFS_DDR_CSR_APB_MT_ADDR_PATTERN);
    }
  else
    {
      putreg32(0, MPFS_DDR_CSR_APB_MT_ADDR_PATTERN);
    }

  if (add_pattern != MTC_ADD_RANDOM)
    {
      /* Set the starting address and number to test
       *
       * MT_START_ADDR
       *   Starting address
       * MT_ADRESS_BITS
       *   Length to test = 2 ** MT_ADRESS_BITS
       */

      putreg32((uint32_t)(start_address & 0xffffffff),
               MPFS_DDR_CSR_APB_MT_START_ADDR_0);

      /* The address here is as see from DDR controller => start at 0x0 */

      putreg32((uint32_t)(start_address >> 32),
               MPFS_DDR_CSR_APB_MT_START_ADDR_1);
    }
  else
    {
      putreg32(0, MPFS_DDR_CSR_APB_MT_START_ADDR_0);
      putreg32(0, MPFS_DDR_CSR_APB_MT_START_ADDR_1);
    }

  putreg32(size, MPFS_DDR_CSR_APB_MT_ADDR_BITS);

  /* MT_ERROR_MASK:
   * All bits set in this field mask corresponding bits in data fields
   * i.e. mt_error and mt_error_hold will not be set for errors in
   * those fields
   *
   * Structure of 144 bits same as DFI bus 36 bits per lane
   * ( 8 physical * 4) + (1ECC * 4) = 36
   *
   * If we wrote out the following pattern from software:
   * 0x12345678
   * 0x87654321
   * 0x56789876
   * 0x43211234
   * We should see:
   *      NNNN_YXXX_XXX3_4YXX_XXXX_76YX_XXXX_X21Y_XXXX_XX78
   *      N: not used
   */

  putreg32(0xffffffff, MPFS_DDR_CSR_APB_MT_ERROR_MASK_0);
  putreg32(0xffffffff, MPFS_DDR_CSR_APB_MT_ERROR_MASK_1);
  putreg32(0xffffffff, MPFS_DDR_CSR_APB_MT_ERROR_MASK_2);
  putreg32(0xffffffff, MPFS_DDR_CSR_APB_MT_ERROR_MASK_3);
  putreg32(0xffffffff, MPFS_DDR_CSR_APB_MT_ERROR_MASK_4);

  if (mask & 0x1)
    {
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_0, 0x000000ff, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_1, 0x00000ff0, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_2, 0x0000ff00, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_3, 0x000ff000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_4, 0x00000000, 0);
    }

  if (mask & 0x2)
    {
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_0, 0x0000ff00, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_1, 0x000ff000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_2, 0x00ff0000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_3, 0x0ff00000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_4, 0x00000000, 0);
    }

  if (mask & 0x4)
    {
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_0, 0x00ff0000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_1, 0x0ff00000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_2, 0xff000000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_3, 0xf0000000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_4, 0x0000000f, 0);
    }

  if (mask & 0x8)
    {
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_0, 0xff000000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_1, 0xf0000000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_2, 0x0000000f, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_3, 0x000000ff, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_4, 0x00000ff0, 0);
    }

  if (mask & 0x10)
    {
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_0, 0x00000000, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_1, 0x0000000f, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_2, 0x000000f0, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_3, 0x00000f00, 0);
      modifyreg32(MPFS_DDR_CSR_APB_MT_ERROR_MASK_4, 0x0000f000, 0);
    }

  /* MT_EN - Enables memory test. If asserted at end of memory test,
   * will keep going.
   */

  putreg32(0, MPFS_DDR_CSR_APB_MT_EN);

  /* MT_EN_SINGLE - Will not repeat if this is set */

  putreg32(0, MPFS_DDR_CSR_APB_MT_EN_SINGLE);
  putreg32(1, MPFS_DDR_CSR_APB_MT_EN_SINGLE);

  /* MT_DONE_ACK - Set when test completes */

  uint32_t retries = MPFS_LONG_RETRIES;

  while (!(getreg32(MPFS_DDR_CSR_APB_MT_DONE_ACK) & 0x01) &&
         --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return -ETIMEDOUT;
    }

  /* Return the error status */

  return getreg32(MPFS_DDR_CSR_APB_MT_ERROR_STS) & 0x01;
}

/****************************************************************************
 * Name: mpfs_set_write_calib
 *
 * Description:
 *   Sets and stores the calibrated values.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a fail.
 *
 ****************************************************************************/

static void mpfs_set_write_calib(struct mpfs_ddr_priv_s *priv)
{
  uint32_t temp = 0;
  uint8_t lane_to_set;
  uint8_t shift = 0;
  uint32_t lanes = priv->number_of_lanes_to_calibrate;

  /* Calculate the calibrated value and write back */

  calib_data.write_cal.lane_calib_result = 0;
  for (lane_to_set = 0; lane_to_set < lanes; lane_to_set++)
    {
      temp = calib_data.write_cal.lower[lane_to_set];
      calib_data.write_cal.lane_calib_result = \
        calib_data.write_cal.lane_calib_result | (temp << (shift));
      shift = (uint8_t)(shift + 0x04);
    }

  /* bit 3 must be set if we want to use the expert_wrcalib
   * register.
   */

  putreg32(0x08, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  /* Set the calibrated value */

  putreg32(calib_data.write_cal.lane_calib_result,
           MPFS_CFG_DDR_SGMII_PHY_EXPERT_WRCALIB);
}

/****************************************************************************
 * Name: mpfs_write_calibration_using_mtc
 *
 * Description:
 *   Use Memory Test Core plugged in to the front end of the DDR controller
 *   to perform lane-based writes and read backs and increment write
 *   calibration offset for each lane until data match occurs. The Memory
 *   Test Core is the basis for all training.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

static int mpfs_write_calibration_using_mtc(struct mpfs_ddr_priv_s *priv)
{
  uint64_t start_address = 0x00;
  uint32_t size          = ONE_MB_MTC;
  uint32_t result        = 0;
  uint8_t lane_to_test;
  uint32_t cal_data;
  uint32_t lanes;

  calib_data.write_cal.status_lower = 0U;

  /* Bit 3 must be set if we want to use the expert_wrcalib register. */

  putreg32(0x08, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  lanes = priv->number_of_lanes_to_calibrate;

  /* Training carried out here: sweeping write calibration offset from 0 to F
   * Explanation: A register, expert_wrcalib, is described in MSS DDR TIP
   * Register Map [1], and its purpose is to delay by X number of memory
   * clock cycles the write data, write data mask, and write output enable
   * with the respect to the address and command for each lane.
   */

  for (cal_data = 0x00000; cal_data < 0xfffff; cal_data += 0x11111)
    {
      putreg32(cal_data, MPFS_CFG_DDR_SGMII_PHY_EXPERT_WRCALIB);

      for (lane_to_test = 0x00; lane_to_test < lanes; lane_to_test++)
        {
          /* Read once to flush MTC. During write calibration the first MTC
           * read must be discarded as it is unreliable after a series of
           * bad writes.
           */

          uint8_t mask = (uint8_t)(1 << lane_to_test);

          result = mpfs_mtc_test(mask, start_address, size,
                        MTC_COUNTING_PATTERN, MTC_ADD_SEQUENTIAL);

          /* Read using different patterns */

          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_COUNTING_PATTERN,
                                  MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_WALKING_ONE, MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_PSEUDO_RANDOM, MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_NO_REPEATING_PSEUDO_RANDOM,
                                  MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_ALT_ONES_ZEROS, MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_ALT_5_A, MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_PSEUDO_RANDOM_16BIT,
                                  MTC_ADD_SEQUENTIAL);
          result |= mpfs_mtc_test(mask, start_address, size,
                                  MTC_PSEUDO_RANDOM_8BIT,
                                  MTC_ADD_SEQUENTIAL);

          if (result == 0) /* if passed for this lane */
            {
              if ((calib_data.write_cal.status_lower &
                  (0x01 << lane_to_test)) == 0)
                {
                  /* Still looking for good value */

                  calib_data.write_cal.lower[lane_to_test] =
                    (cal_data & 0xf);
                  calib_data.write_cal.status_lower       |=
                    (0x01 << lane_to_test);
                }

              /* Check the result */

              uint32_t lane_to_check;

              for (lane_to_check = 0; lane_to_check < lanes;
                   lane_to_check++)
                {
                  if (((calib_data.write_cal.status_lower) &
                       (0x01 << lane_to_check)) == 0)
                    {
                      /* not finished, still looking */

                      result = 1;
                      break;
                    }
                }

              if (result == 0)
                {
                  /* We're good for all lanes, can stop */

                  break;
                }
            }
        }

      if (result == 0)
        {
          /* if true, we are good for all lanes, can stop searching */

          break;
        }
    }

  /* If calibration successful, calculate and set the value */

  if (result == 0)
    {
      /* Set the write calibration which has been calculated */

      mpfs_set_write_calib(priv);
    }

  return result;
}

/****************************************************************************
 * Name: mpfs_ddr_rand
 *
 * Description:
 *   This should return a random value.
 *
 * Returned Value:
 *   Always zero at the moment.
 *
 * Assumptions/Limitations:
 *   This doesn't return random values at the moment.
 *
 ****************************************************************************/

static int mpfs_ddr_rand(void)
{
  return 0;
}

/****************************************************************************
 * Name: mpfs_ddr_write
 *
 * Description:
 *   This writes values into DDR with various patterns. These values are
 *   meant to be read back and validated later.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *   ddr_word_ptr  - DDR address to start from
 *   no_of_access  - Number of accesses
 *   data_ptrn     - Data pattern
 *   data_width    - 64 or 32 bit data width
 *
 ****************************************************************************/

static void mpfs_ddr_write(struct mpfs_ddr_priv_s *priv,
                           volatile uint64_t *ddr_word_ptr,
                           uint32_t no_of_access,
                           int8_t data_ptrn,
                           enum ddr_access_size_e data_width)
{
  uint64_t data;
  uint32_t i;

  volatile uint32_t *ddr_32_ptr = (uint32_t *)ddr_word_ptr;

  switch (data_ptrn)
    {
      case PATTERN_INCREMENTAL:
        data = 0x00000000;
        break;

      case PATTERN_WALKING_ONE:
        data = 0x00000001;
        break;

      case PATTERN_WALKING_ZERO:
        data = 0x01;
        data = ~data;
        break;

      case PATTERN_RANDOM:
        data = (uint64_t)mpfs_ddr_rand();
        break;

      case PATTERN_CCCCCCCC:
        data = 0xcccccccccccccccc;
        break;

      case PATTERN_55555555:
        data = 0x5555555555555555;
        break;

      case PATTERN_ZEROS:
        data = 0x00000000;
        break;

      default:
        data = 0x00000000;
        break;
    }

  for (i = 0; i < no_of_access; i++)
    {
      switch (data_width)
        {
          case DDR_32_BIT:
            data &= 0xffffffff;
            *ddr_32_ptr = (uint32_t)data;
            ddr_32_ptr = ddr_32_ptr + 1;
            break;

          default:
          case DDR_64_BIT:
            *ddr_word_ptr = data;
            ddr_word_ptr = ddr_word_ptr + 1;
            break;
        }

      switch (data_ptrn)
        {
          case PATTERN_INCREMENTAL:
            data = data + 0x00000001;
            break;

          case PATTERN_WALKING_ONE:
            if (data == 0x80000000)
              {
                data = 0x00000001;
              }
            else
              {
                data = (data << 1);
              }
            break;

          case PATTERN_WALKING_ZERO:
            data = ~data;
            if (data == 0x80000000)
              data = 0x00000001;
            else
              {
                data = (data << 1);
              }

            data = ~data;
            break;

          case PATTERN_RANDOM:
            data = (uint64_t)mpfs_ddr_rand();
            break;

          case PATTERN_CCCCCCCC:
            data = 0xcccccccccccccccc;
            break;

          case PATTERN_55555555:
            data = 0x5555555555555555;
            break;

          case PATTERN_ZEROS:
            data = 0x00000000;
            break;

          default:
            break;
        }
    }
}

/****************************************************************************
 * Name: mpfs_ddr_read
 *
 * Description:
 *   This reads back values from DDR written earlier by mpfs_ddr_write() and
 *   validates them.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *   ddr_word_ptr  - DDR address to start from
 *   no_of_access  - Number of accesses
 *   data_ptrn     - Data pattern
 *   data_width    - 64 or 32 bit data width
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

uint32_t mpfs_ddr_read(struct mpfs_ddr_priv_s *priv,
                       volatile uint64_t *ddr_word_ptr,
                       uint32_t no_of_access,
                       uint8_t data_ptrn,
                       enum ddr_access_size_e data_width)
{
  uint32_t i;
  uint64_t data;
  uint32_t err_cnt;
  volatile uint64_t ddr_data;
  volatile uint64_t *ddr_word_pt_t;
  volatile uint64_t *first_ddr_word_pt_t;
  uint32_t mpfs_ddr_rand_addr_offset;
  volatile uint32_t *ddr_32_pt_t;

  err_cnt = 0U;
  first_ddr_word_pt_t = ddr_word_ptr;
  ddr_32_pt_t = (uint32_t *)ddr_word_ptr;

  switch (data_ptrn)
    {
      case PATTERN_INCREMENTAL:
        data = 0x00000000;
        break;

      case PATTERN_WALKING_ONE:
        data = 0x00000001;
        break;

      case PATTERN_WALKING_ZERO:
        data = 0x01;
        data = ~data;
        break;

      case PATTERN_RANDOM:
        data = (uint64_t)mpfs_ddr_rand();
        *ddr_word_ptr = data;
        *ddr_32_pt_t = (uint32_t)data;
        break;

      case PATTERN_CCCCCCCC:
        data = 0xcccccccccccccccc;
        break;

      case PATTERN_55555555:
        data = 0x5555555555555555;
        break;

      case PATTERN_ZEROS:
        data = 0x00000000;
        break;

      default:
        data = 0x00000000;
        break;
    }

  if (data_ptrn == '4')
    {
        mpfs_wait_cycles(10);
    }

  for (i = 0; i < (no_of_access); i++)
    {
      switch (data_width)
        {
          case DDR_32_BIT:
            data &= 0xffffffff;
            ddr_data = *ddr_32_pt_t;
            break;

          default:
          case DDR_64_BIT:
            ddr_word_pt_t = ddr_word_ptr;
            ddr_data = *ddr_word_pt_t;
            break;
        }

      if (ddr_data != data)
        {
          err_cnt++;
        }

      ddr_word_ptr = ddr_word_ptr + 1U;
      ddr_32_pt_t  = ddr_32_pt_t +1U;

      switch (data_ptrn)
        {
          case PATTERN_INCREMENTAL:
            data = data + 0x01;
            break;

          case PATTERN_WALKING_ONE:
            if (data == 0x80000000)
              data = 0x00000001;
            else
               data = (data << 1);
            break;

            case PATTERN_WALKING_ZERO:
              data = ~data;
              if (data == 0x80000000)
                {
                  data = 0x00000001;
                }
              else
                {
                  data = (data << 1);
                }

              data = ~data;
              break;

            case PATTERN_RANDOM:
                data = (uint64_t)mpfs_ddr_rand();
                mpfs_ddr_rand_addr_offset = (uint32_t)(mpfs_ddr_rand() &
                                             0xffffc);
                ddr_word_ptr = first_ddr_word_pt_t +
                               mpfs_ddr_rand_addr_offset;
                ddr_32_pt_t  = (uint32_t *)(first_ddr_word_pt_t +
                               mpfs_ddr_rand_addr_offset);
                *ddr_word_ptr   = data;
                *ddr_32_pt_t    = (uint32_t)data;
                break;
            case PATTERN_CCCCCCCC:
                data = 0xcccccccccccccccc;
                break;
            case PATTERN_55555555:
                data = 0x5555555555555555;
                break;
            case PATTERN_ZEROS:
                data = 0x00000000;
                break;
            default:
                break;
          }
    }

  return (err_cnt);
}

/****************************************************************************
 * Name: mpfs_ddr_read_write_fn
 *
 * Description:
 *   This writes and reads back values from the determined location in DDR
 *   memory.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *   ddr_word_ptr  - DDR address to start from
 *   no_access     - Number of accesses
 *   pattern       - Data pattern
 *
 * Returned Value:
 *   0 on success, number of errors otherwise.
 *
 ****************************************************************************/

static uint32_t mpfs_ddr_read_write_fn(struct mpfs_ddr_priv_s *priv,
                                       uint64_t *ddr_word_ptr,
                                       uint32_t no_access,
                                       uint32_t pattern)
{
  uint32_t error_cnt = 0;
  uint8_t pattern_mask;
  uint32_t pattern_shift;

  for (pattern_shift = 0; pattern_shift < MAX_NO_PATTERNS;
       pattern_shift++)
    {
      pattern_mask = (uint8_t)(0x01 << pattern_shift);

      if (pattern & pattern_mask)
        {
          /* Write the pattern */

          mpfs_ddr_write(priv, (uint64_t *)ddr_word_ptr, no_access,
                         pattern_mask, DDR_64_BIT);

          error_cnt += mpfs_ddr_read(priv, (uint64_t *)ddr_word_ptr,
                                     no_access, pattern_mask,
                                     DDR_64_BIT);

          /* Write the pattern */

          mpfs_ddr_write(priv, (uint64_t *)ddr_word_ptr, no_access,
                                pattern_mask, DDR_32_BIT);

          /* Read back and verifies */

          error_cnt += mpfs_ddr_read(priv, (uint64_t *)ddr_word_ptr,
                                     no_access, pattern_mask,
                                     DDR_32_BIT);
        }
    }

  return error_cnt;
}

/****************************************************************************
 * Name: mpfs_ddr_manual_addcmd_training
 *
 * Description:
 *   Performs manual addcmd training.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure.
 *
 ****************************************************************************/

static void mpfs_ddr_manual_addcmd_training(struct mpfs_ddr_priv_s *priv)
{
  uint32_t bclk_phase;
  uint32_t bclk90_phase;
  uint32_t refclk_phase;
  uint32_t ca_indly;
  uint32_t dpc_vals;
  uint32_t j;
  uint32_t i;

  /* Apply offset & load the phase */

  bclk_phase = ((priv->bclk_answer + SW_TRAINING_BCLK_SCLK_OFFSET) &
                0x07) << 8;
  bclk90_phase = ((priv->bclk_answer + SW_TRAINING_BCLK_SCLK_OFFSET + 2) &
                  0x07) << 11;

  putreg32((0x00004003 | bclk_phase | bclk90_phase),
           MPFS_IOSCB_DDR_PLL_PHADJ);
  putreg32((0x00000003 | bclk_phase | bclk90_phase),
           MPFS_IOSCB_DDR_PLL_PHADJ);
  putreg32((0x00004003 | bclk_phase | bclk90_phase),
           MPFS_IOSCB_DDR_PLL_PHADJ);

  /* Store DRV & VREF initial values (to be re-applied after
   * CA training)
   */

  uint32_t ca_drv = getreg32(MPFS_CFG_DDR_SGMII_PHY_RPC1_DRV);
  uint32_t ca_vref = (getreg32(MPFS_CFG_DDR_SGMII_PHY_DPC_BITS) >> 12) &
                      0x3f;

  uint32_t dpc_bits_new;
  uint32_t vref_answer;
  uint32_t transition_a5_min_last = 129;

  putreg32(0x01, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  for (ca_indly = 0; ca_indly < 30; ca_indly = ca_indly + 5)
    {
      putreg32(ca_indly, MPFS_CFG_DDR_SGMII_PHY_RPC145);
      putreg32(ca_indly, MPFS_CFG_DDR_SGMII_PHY_RPC147);
      uint32_t break_loop = 1;
      uint32_t in_window = 0;
      vref_answer = 128;

      /* Begin VREF training */

      for (uint32_t vref = 5; vref < 30; vref++)
        {
          uint32_t transition_a5_max = 0;
          uint32_t transition_a5_min = 128;
          uint32_t rx_a5_last;
          uint32_t rx_a5;
          uint32_t transition_a5;
          uint32_t range_a5 = 0;

          if (transition_a5_min_last > 128)
            {
              transition_a5_min_last = 128;
            }

          putreg32(0, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);

          /* Set VREF */

          mpfs_wait_cycles(10);
          dpc_bits_new = (getreg32(MPFS_CFG_DDR_SGMII_PHY_DPC_BITS) &
                          0xfffc0fff) | (vref << 12) | (0x1 << 18);
          putreg32(dpc_bits_new, MPFS_CFG_DDR_SGMII_PHY_DPC_BITS);
          mpfs_wait_cycles(10);
          putreg32(1, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);
          mpfs_wait_cycles(10);

          uint32_t deltat = 128;

          for (j = 0; j < 20; j++)
            {
              putreg32(0x00,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
              putreg32(0x180000,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

              putreg32(0x180000,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
              putreg32(0x180000,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

              rx_a5_last = 0xf;
              transition_a5 = 0;
              deltat = 128;
              mpfs_wait_cycles(10);

              for (i = 0; i < (128 - ca_indly); i++)
                {
                  putreg32(0x00,
                           MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
                  putreg32(0x180000,
                           MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
                  putreg32(0x00,
                           MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
                  mpfs_wait_cycles(10);

                  rx_a5 = (getreg32(
                          MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK) \
                          & 0x0300) >> 8;

                  if (transition_a5 != 0)
                    {
                      if (((i - transition_a5) > 8))
                        {
                          break;
                        }
                    }

                  if (transition_a5 == 0)
                    {
                      if (((rx_a5 ^ rx_a5_last) & rx_a5))
                        {
                          transition_a5 = i;
                        }
                      else
                        {
                          rx_a5_last = rx_a5;
                        }
                    }
                  else
                    {
                      if ((i - transition_a5) == 4)
                        {
                          if (!((rx_a5 ^ rx_a5_last) & rx_a5))
                            {
                              /* Continue looking for transition */

                              transition_a5 = 0;
                              rx_a5_last = rx_a5;
                            }
                        }
                    }
                }

              if (transition_a5 != 0)
                {
                  if (transition_a5 > transition_a5_max)
                    {
                      transition_a5_max = transition_a5;
                    }

                  if (transition_a5 < transition_a5_min)
                    {
                      transition_a5_min = transition_a5;
                    }
                }
            }

          range_a5 = transition_a5_max - transition_a5_min;

          if (transition_a5_min < 10)
            {
              break_loop = 0;
            }

          if (range_a5 <= 5)
            {
              if (transition_a5_min > transition_a5_min_last)
                {
                  deltat = transition_a5_min - transition_a5_min_last;
                }
              else
                {
                  deltat = transition_a5_min_last - transition_a5_min;
                }

              if (deltat <= 5)
                {
                  in_window = (in_window << 1) | 1;
                }
            }
          else
            {
              in_window = (in_window << 1) | 0;
            }

          if (vref_answer == 128)
            {
              if ((in_window & 0x3) == 0x3)
                {
                  vref_answer = vref;
                  break;
                }
            }

          transition_a5_min_last = transition_a5_min;
        }

      if (break_loop)
        {
          break;
        }
    }

  putreg32(0, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);

  /* Set VREF */

  mpfs_wait_cycles(10);

  if (vref_answer == 128)
    {
      vref_answer = 0x10;
      dpc_bits_new = (getreg32(MPFS_CFG_DDR_SGMII_PHY_DPC_BITS) & 0xfffc0fff)
                      | (vref_answer << 12) | (0x1 << 18);
    }
  else
    {
      vref_answer = vref_answer;
      dpc_bits_new = (getreg32(MPFS_CFG_DDR_SGMII_PHY_DPC_BITS) & 0xfffc0fff)
                      | (vref_answer << 12) | (0x1 << 18);
    }

  putreg32(dpc_bits_new, MPFS_CFG_DDR_SGMII_PHY_DPC_BITS);
  mpfs_wait_cycles(10);
  putreg32(1, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);
  mpfs_wait_cycles(10000);

  /* Begin manual addcmd training */

  uint32_t init_del_offset = 0x8;
  uint32_t a5_offset_fail;
  uint32_t rpc147_offset = 0x2;
  uint32_t rpc145_offset = 0x0;
  uint8_t refclk_offset = mpfs_ddr_manual_addcmd_refclk_offset(priv);
  a5_offset_fail = 1;

  while (a5_offset_fail)
    {
      a5_offset_fail = 0; /* 1 indicates a fail */

      putreg32(init_del_offset + rpc147_offset,
               MPFS_CFG_DDR_SGMII_PHY_RPC147);
      putreg32(init_del_offset + rpc145_offset,
               MPFS_CFG_DDR_SGMII_PHY_RPC145);

      putreg32(0x03, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

      uint32_t rx_a5;
      uint32_t rx_a5_last;
      uint32_t rx_ck;
      uint32_t rx_ck_last;
      uint32_t transition_a5;
      uint32_t transition_ck;
      uint32_t difference[8] = {
        0
      };

      uint32_t transition_ck_array[8] = {
        0
      };

      uint32_t transitions_found;
      uint32_t transition_a5_max = 0;

      for (j = 0; j < 16; j++)
        {
          /* Increase j loop to increase number of samples on transition_a5
           * (for noisy CA in LPDDR4)
           */

          /* Load INDLY */

          putreg32(0x00,
                   MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);

          putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x180000, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

          /* Load OUTDLY */

          putreg32(0x180000,
                   MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);

          putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x180000, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

          refclk_phase = (j % 8U) << 2U;
          putreg32((0x00004003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);
          putreg32((0x00000003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);
          putreg32((0x00004003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);

          rx_a5_last = 0xf;
          rx_ck_last = 0x5;
          transition_a5 = 0;
          transition_ck = 0;

          mpfs_wait_cycles(100);
          transitions_found = 0;
          i = 0;

          while ((!transitions_found) & (i < 128))
            {
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
              putreg32(0x180000,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
              putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
              mpfs_wait_cycles(10);

              rx_a5 =
                (getreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK) &
                 0x0300) >> 8;
              rx_ck =
                getreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK) &
                0x000f;

              if ((transition_a5 != 0) && (transition_ck != 0))
                {
                  if (((i - transition_a5) > 8) && ((i - transition_ck) > 8))
                    {
                      transitions_found = 1;
                    }
                }

              if (transition_ck == 0)
                {
                  if (rx_ck_last != 0x5) /* If edge detected */
                    {
                      if (rx_ck == 0x5)
                        transition_ck = i; /* Transition detected at i */
                    }

                  rx_ck_last = rx_ck;
                }
              else
                {
                  if ((i - transition_ck) == 4)
                    {
                      /* If rx_ck not stable after 4 increments, mark it
                       * a false transition
                       */

                      if (rx_ck != rx_ck_last)
                        {
                          /* Keep looking for transition */

                          transition_ck = 0;
                          rx_ck_last = rx_ck;
                        }
                    }
                }

              if (transition_a5 == 0)
                {
                  if (((rx_a5 ^ rx_a5_last) & rx_a5))
                    {
                      transition_a5 = i;
                    }
                  else
                    {
                      rx_a5_last = rx_a5;
                    }
                }
              else
                {
                  if ((i - transition_a5) == 4)
                    {
                      if (!((rx_a5 ^ rx_a5_last) & rx_a5))
                        {
                          /* Keep looking for transition */

                          transition_a5 = 0;
                          rx_a5_last = rx_a5;
                        }
                    }
                }

              if ((transition_a5 != 0) && (transition_ck != 0))
                {
                  if ((i == transition_a5) || (i == transition_ck))
                    {
                    }
                }

              i++;
            }

          if (transition_a5 > transition_a5_max)
            {
              transition_a5_max = transition_a5;
            }

          if ((transition_a5 != 0) && (transition_ck != 0) && (j < 8))
            {
              transition_ck_array[j] = transition_ck;
            }
        }

      uint32_t min_diff = 0xff;
      uint32_t min_refclk = 0x8;

      if (transition_a5_max < 5)
        {
          a5_offset_fail = a5_offset_fail | 1;
        }

      for (uint32_t k = 0; k < 8; k++)
        {
          if (transition_a5_max >= transition_ck_array[k])
            difference[k] = transition_a5_max - transition_ck_array[k];
          else
            difference[k] = 0xff;

          if (difference[k] < min_diff)
            {
              min_diff = difference[k];
              min_refclk = k;
            }
        }

      if (min_diff == 0xff)
        {
          a5_offset_fail = a5_offset_fail | 1;
        }

      if (min_refclk == 0x8)
        {
          /* If ADDCMD training fails due to extremely low frequency,
           * use PLL to provide offset.
           */

          a5_offset_fail = a5_offset_fail | 4;
        }

      if (a5_offset_fail == 0)
        {
          refclk_phase = ((refclk_offset + min_refclk) & 0x7) << 2;

          putreg32((0x00004003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);
          putreg32((0x00000003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);
          putreg32((0x00004003 | bclk_phase | bclk90_phase | refclk_phase),
                   MPFS_IOSCB_DDR_PLL_PHADJ);

          /* Load INDLY */

          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);

          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x180000, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

          /* Load OUTDLY */

          putreg32(0x180000,
                   MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);
          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x180000, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

          for (uint32_t m = 0; m < min_diff; m++)
            {
              putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
              putreg32(0x180000,
                       MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
              putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1);
            }

          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1);
          putreg32(0x0, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);
        }
      else
        {
          if (a5_offset_fail & 0x1)
            {
              if (init_del_offset < 0xff)
                {
                  /* If transition_a5 too low, increase indly offset on CK
                   * and CA and retrain
                   */

                  init_del_offset += (transition_a5_max) + 5;
                }
              else
                {
                  break;
                }
            }
        }
    }

  /* Set VREF back to configured value */

  putreg32(0, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);
  mpfs_wait_cycles(10);

  dpc_vals = (getreg32(MPFS_CFG_DDR_SGMII_PHY_DPC_BITS) & 0xfffc0fff) |
              (ca_vref << 12) | (0x1 << 18);
  putreg32(dpc_vals, MPFS_CFG_DDR_SGMII_PHY_DPC_BITS);

  mpfs_wait_cycles(10);

  putreg32(0x01, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);
  mpfs_wait_cycles(10);

  /* Set CA DRV back to configured value */

  putreg32(ca_drv, MPFS_CFG_DDR_SGMII_PHY_RPC1_DRV);
}

/****************************************************************************
 * Name: mpfs_ddr_sm_init
 *
 * Description:
 *   Initializes the state machine. In the event on -EAGAIN later, this will
 *   be called again before restarting the entire process.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *
 ****************************************************************************/

static void mpfs_ddr_sm_init(struct mpfs_ddr_priv_s *priv)
{
  priv->ddr_type            = LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_MASK;
  priv->write_latency       = LIBERO_SETTING_CFG_WRITE_LATENCY_SET;
  priv->tip_cfg_params      = LIBERO_SETTING_TIP_CFG_PARAMS;
  priv->dpc_bits            = LIBERO_SETTING_DPC_BITS;
  priv->rpc_166_fifo_offset = DEFAULT_RPC_166_VALUE;
  priv->error               = 0;
  priv->retry_count         = 0;
  priv->num_rpc_166_retires = 0;
  priv->refclk_sweep_index  = 0xf;

  priv->number_of_lanes_to_calibrate = mpfs_get_num_lanes();

  memset(&calib_data, 0, sizeof(calib_data));
}

/****************************************************************************
 * Name: mpfs_ddr_fail
 *
 * Description:
 *   This performs the mandatory steps to close-up the ddr controller if
 *   the training failed on a permanent manner.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *
 ****************************************************************************/

static void mpfs_ddr_fail(struct mpfs_ddr_priv_s *priv)
{
  memset(&calib_data, 0, sizeof(calib_data));

  putreg32(0, MPFS_DDR_CSR_APB_PHY_DFI_INIT_START);

  /* Reset controller */

  putreg32(0, MPFS_DDR_CSR_APB_CTRLR_INIT);
  putreg32(0, MPFS_CFG_DDR_SGMII_PHY_TRAINING_START);

  if (priv->ddr_type == DDR_OFF_MODE)
    {
      mpfs_ddr_off_mode();
    }
}

/****************************************************************************
 * Name: mpfs_set_mode_vs_bits
 *
 * Description:
 *   Select VS bits for DDR mode selected - set dynamic pc bit settings to
 *   allow editing of RPC registers pvt calibration etc.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

static int mpfs_set_mode_vs_bits(struct mpfs_ddr_priv_s *priv)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;
  uint32_t d;

  /* Set the training mode */

  mpfs_set_ddr_mode_reg_and_vs_bits(priv);

  /* Flash registers with RPC values */

  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_DRIVER);
  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_ODT);
  putreg32(1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_IO);

  /* Correct some rpc registers, which were incorrectly set in mode
   * setting
   */

  mpfs_set_ddr_rpc_regs(priv);

  /* Set soft reset on IP to load RPC to SCB regs (dynamic mode),
   * bring the DDR bank controller out of reset
   */

  putreg32(1, MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET);

  /* Calibrate DDR I/O here, once all RPC settings correct */

  if (mpfs_ddr_pvt_calibration())
    {
      return 1;
    }

  /* Configure the DDR PLL */

  mpfs_ddr_pll_config();

  /* Verify DDR PLL lock */

  while (mpfs_ddr_pll_lock_scb() && --retries);

  if (retries == 0)
    {
      merr("Timeout\n");
      return 1;
    }

  /* Configure Segments- address mapping, CFG0/CFG1 */

  mpfs_setup_ddr_segments(DEFAULT_SEG_SETUP);

  /* Turn on DDRC clock */

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
              SYSREG_SUBBLK_CLOCK_CR_DDRC);

  /* Remove soft reset */

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SUBBLK_CLOCK_CR_DDRC, 0);

  /* Set-up DDRC */

  mpfs_init_ddrc();

  /* Assert training reset, reset pin is bit 1 */

  putreg32(0x02, MPFS_CFG_DDR_SGMII_PHY_TRAINING_RESET);

  putreg32(0x00, MPFS_DDR_CSR_APB_CTRLR_SOFT_RESET_N);
  putreg32(0x01, MPFS_DDR_CSR_APB_CTRLR_SOFT_RESET_N);

  /* Rotate bclk90 by 90 deg */

  putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);

  /* Expert mode enable */

  putreg32(0x02, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  putreg32(0x7c, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x78, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x78, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x7c, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x64, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x66, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);

  for (d = 0; d < LIBERO_SETTING_TIP_CONFIG_PARAMS_BCLK_VCOPHS_OFFSET; d++)
    {
      putreg32(0x67, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
      putreg32(0x66, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
    }

  putreg32(0x64, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);
  putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT);

  /* setting load delay lines */

  putreg32(0x1f, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MV_RD_DLY_REG);
  putreg32(0xffffffff, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MV_RD_DLY_REG);
  putreg32(0xffffffff, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

  putreg32(0x3f, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE);

  putreg32(0x06, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE_OFFSET);

  putreg32(0xffffffff, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
  putreg32(0x0f, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

  putreg32(0x04, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE_OFFSET);

  putreg32(0xffffffff, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
  putreg32(0x0f, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1);

  /* Clear */

  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE_OFFSET);
  putreg32(0x3f, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE);
  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN);

  /* Set training parameters
   *
   * Tip static parameters 0:
   *
   *  30:22   Number of VCO Phase offsets between BCLK and SCLK
   *  21:13   Number of VCO Phase offsets between BCLK and SCLK
   *  12:6    Number of VCO Phase offsets between BCLK and SCLK
   *  5:3     Number of VCO Phase offsets between BCLK and SCLK
   *  2:0     Number of VCO Phase offsets between REFCLK and ADDCMD bits
   */

  putreg32(priv->tip_cfg_params, MPFS_CFG_DDR_SGMII_PHY_TIP_CFG_PARAMS);
  priv->timeout = 0xffff;

  return 0;
}

/****************************************************************************
 * Name: mpfs_bclksclk_sw
 *
 * Description:
 *   This performs a manual BCLKSCLK training.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *
 ****************************************************************************/

static void mpfs_bclksclk_sw(struct mpfs_ddr_priv_s *priv)
{
  uint32_t rx_previous = 0x3;
  uint32_t rx_current = 0;
  uint32_t answer_count[8] = {
    0
  };

  uint32_t answer_index = 0;
  uint32_t bclk90_phase;
  uint32_t bclk_phase;
  uint32_t max;
  uint32_t j;
  uint32_t i;

  /* We have chosen to use software bclk sclk sweep instead of IP */

  priv->bclk_answer = 0;

  for (i = 0; i < (8 * 100); i++)
    {
      bclk_phase = (i & 0x07) << 8;
      bclk90_phase = ((i + 2) & 0x07) << 11;

      /* Load BCLK90 phase */

      putreg32((0x00004003 | bclk_phase | bclk90_phase),
               MPFS_IOSCB_DDR_PLL_PHADJ);
      putreg32((0x00000003 | bclk_phase | bclk90_phase),
               MPFS_IOSCB_DDR_PLL_PHADJ);
      putreg32((0x00004003 | bclk_phase | bclk90_phase),
               MPFS_IOSCB_DDR_PLL_PHADJ);

      /* Sample RX_BCLK */

      rx_current = (
                  getreg32(MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK)
                  >> 12) & 0x03;

      /* If transition found, break the loop */

      if ((rx_current & (~rx_previous)) != 0x00)
        {
          answer_index = i & 0x07;

          /* Increment the answer count for this index */

          answer_count[answer_index]++;
        }

      rx_previous = rx_current;
      max = 0;

      for (j = 0; j < 8; j++)
        {
          /* Sweep through found answers and select the most common */

          if (answer_count[j] > max)
            {
              priv->bclk_answer = j;
              max = answer_count[j];
            }
        }
    }
}

/****************************************************************************
 * Name: mpfs_training_start
 *
 * Description:
 *   This starts the DDR training process.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 ****************************************************************************/

static void mpfs_training_start(struct mpfs_ddr_priv_s *priv)
{
  putreg32(LIBERO_SETTING_TRAINING_SKIP_SETTING,
           MPFS_CFG_DDR_SGMII_PHY_TRAINING_SKIP);

  if ((priv->ddr_type == DDR3) || (priv->ddr_type == LPDDR4) ||
      (priv->ddr_type == DDR4))
    {
      /* RX_MD_CLKN */

      putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_RPC168);
    }

  /* Release reset to training */

  putreg32(0x00, MPFS_CFG_DDR_SGMII_PHY_TRAINING_RESET);

  putreg32(0, MPFS_DDR_CSR_APB_PHY_DFI_INIT_START);

  putreg32(1, MPFS_DDR_CSR_APB_PHY_DFI_INIT_START);
  putreg32(0, MPFS_DDR_CSR_APB_CTRLR_INIT);
  putreg32(1, MPFS_DDR_CSR_APB_CTRLR_INIT);

  priv->timeout = 0xffff;
}

/****************************************************************************
 * Name: mpfs_training_start_check
 *
 * Description:
 *   Checks the training has completed.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

static int mpfs_training_start_check(struct mpfs_ddr_priv_s *priv)
{
  unsigned int retries = MPFS_DEFAULT_RETRIES;

  while (!(getreg32(MPFS_DDR_CSR_APB_STAT_DFI_INIT_COMPLETE) & 0x01) &&
         --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_bclksclk
 *
 * Description:
 *   Checks the BCLK/SCLK training status for completion.
 *
 * Input Parameters:
 *   priv          - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

static int mpfs_training_bclksclk(struct mpfs_ddr_priv_s *priv)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS) & BCLK_SCLK_BIT)
         && --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_addcmd
 *
 * Description:
 *   Checks the training addcmd status for completion.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure
 *
 ****************************************************************************/

static int mpfs_training_addcmd(void)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  if (LIBERO_SETTING_TRAINING_SKIP_SETTING & ADDCMD_BIT)
    {
      return 0;
    }

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS) & ADDCMD_BIT)
         && --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_verify
 *
 * Description:
 *   Checks the training addcmd status for completion and read the training
 *   results.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A nonzero value indicates a failure.
 *
 ****************************************************************************/

static int mpfs_training_verify(void)
{
  uint32_t low_ca_dly_count;
  uint32_t decrease_count;
  uint32_t addcmd_status0;
  uint32_t addcmd_status1;
  uint32_t retries = MPFS_DEFAULT_RETRIES;
  uint32_t t_status = 0;
  uint32_t lane_sel;
  uint32_t last;
  uint32_t i;

  while (!(getreg32(MPFS_DDR_CSR_APB_STAT_DFI_TRAINING_COMPLETE) & 0x01) &&
        --retries);

  if (retries == 0)
    {
      merr("Timeout\n");
      return -ETIMEDOUT;
    }

  for (lane_sel = 0; lane_sel < LIBERO_SETTING_DATA_LANES_USED; lane_sel++)
    {
      mpfs_wait_cycles(10);

      putreg32(lane_sel, MPFS_CFG_DDR_SGMII_PHY_LANE_SELECT);
      mpfs_wait_cycles(10);

      /* Verify cmd address results, rejects if not acceptable */

      addcmd_status0 = getreg32(MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS0);
      addcmd_status1 = getreg32(MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS1);

      uint32_t ca_status[8] =
        {
          ((addcmd_status0) & 0xff),
          ((addcmd_status0 >> 8) & 0xff),
          ((addcmd_status0 >> 16) & 0xff),
          ((addcmd_status0 >> 24) & 0xff),
          ((addcmd_status1) & 0xff),
          ((addcmd_status1 >> 8) & 0xff),
          ((addcmd_status1 >> 16) & 0xff),
          ((addcmd_status1 >> 24) & 0xff)
        };

      low_ca_dly_count = 0;
      last             = 0;
      decrease_count   = 0;

      for (i = 0; i < 8; i++)
        {
          if (ca_status[i] < 5)
            {
              low_ca_dly_count++;
            }

          if (ca_status[i] <= last)
            {
              decrease_count++;
            }

          last = ca_status[i];
        }

      if (ca_status[0] <= ca_status[7])
        {
          decrease_count++;
        }

      if ((LIBERO_SETTING_TRAINING_SKIP_SETTING & ADDCMD_BIT) != ADDCMD_BIT)
        {
          /* Retrain if abnormal CA training result detected */

          if (low_ca_dly_count > ABNORMAL_RETRAIN_CA_DLY_DECREASE_COUNT)
            {
              t_status |= 0x01;
            }

          /* Retrain if abnormal CA training result detected */

          if (decrease_count > ABNORMAL_RETRAIN_CA_DECREASE_COUNT)
            {
              t_status |= 0x01;
            }
        }

      /* Check that gate training passed without error  */

      t_status |= getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_ERR_COMB);
      mpfs_wait_cycles(10);

      /* Check that DQ/DQS training passed without error */

      if (getreg32(MPFS_CFG_DDR_SGMII_PHY_DQ_DQS_ERR_DONE) != 8)
        {
          t_status |= 0x01;
        }

      /* Check that DQ/DQS calculated window is above 5 taps. */

      if (getreg32(MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS1) < DQ_DQS_NUM_TAPS)
        {
          t_status |= 0x01;
        }

      /* Extra checks */

      uint32_t temp = 0;
      uint32_t gt_clk_sel = getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_CLK_SEL) &
                                     0x03;

      if ((getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY) & 0xff) == 0)
        {
          temp++;
          if (gt_clk_sel == 0)
            {
              t_status |= 0x01;
            }
        }

      if (((getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY) >> 8) & 0xff) == 0)
        {
          temp++;
          if (gt_clk_sel == 1)
            {
              t_status |= 0x01;
            }
        }

      if (((getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY) >> 16) & 0xff) == 0)
        {
          temp++;
          if (gt_clk_sel == 2)
            {
              t_status |= 0x01;
            }
        }

      if (((getreg32(MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY) >> 24) & 0xff) == 0)
        {
          temp++;
          if (gt_clk_sel == 3)
            {
              t_status |= 0x01;
            }
        }

      if (temp > 1)
        {
          t_status |= 0x01;
        }
    }

  if (t_status != 0)
    {
      merr("Training failed. Restarting!\n");
      return -EAGAIN;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_wrlvl_wait
 *
 * Description:
 *   Waits for the WRLVL bit being set.
 *
 * Returned Value:
 *   Zero is returned on success. -ETIMEDOUT otherwise.
 *
 ****************************************************************************/

static int mpfs_training_wrlvl_wait(void)
{
  uint32_t retries = MPFS_LONG_RETRIES;

  if (LIBERO_SETTING_TRAINING_SKIP_SETTING & WRLVL_BIT)
    {
      return 0;
    }

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS) & WRLVL_BIT) &&
         --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_rdgate
 *
 * Description:
 *   Checks for the read gate training completion.
 *
 * Returned Value:
 *   Zero is returned on success. -ETIMEDOUT otherwise.
 *
 ****************************************************************************/

static int mpfs_training_rdgate(void)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  if (LIBERO_SETTING_TRAINING_SKIP_SETTING & RDGATE_BIT)
    {
      return 0;
    }

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS) & RDGATE_BIT) &&
         --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return -ETIMEDOUT;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_dq_dqs
 *
 * Description:
 *   Checks for the DQ/DQS training completion.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero is returned on success. -ETIMEDOUT otherwise.
 *
 ****************************************************************************/

static int mpfs_dq_dqs(void)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS) & DQ_DQS_BIT) &&
         --retries);

  if (retries == 0)
    {
      merr("Timeout!\n");
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_write_calibration
 *
 * Description:
 *   Checks for the DQ/DQS training completion
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero is returned on success. Nonzero indicates a failure
 *
 ****************************************************************************/

static int mpfs_training_write_calibration(struct mpfs_ddr_priv_s *priv)
{
  uint32_t nr_lanes = mpfs_get_num_lanes();
  int error;

  /* Now start the write calibration as training has been successful */

  if (priv->ddr_type == LPDDR4)
    {
      uint8_t lane;

      /* Changed default value to centre dq/dqs on window */

      putreg32(0x0c, MPFS_CFG_DDR_SGMII_PHY_RPC220);

      for (lane = 0; lane < nr_lanes; lane++)
        {
          mpfs_load_dq(lane);
        }

      error = mpfs_write_calibration_using_mtc(priv);
    }
  else
    {
      error = mpfs_write_calibration_using_mtc(priv);
    }

  if (error)
    {
      merr("Will retry..\n");
      return -EAGAIN;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_write_calib_retry
 *
 * Description:
 *   Increases the write latency value before retrying the process
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero is returned on success. Nonzero indicates a failure
 *
 ****************************************************************************/

static int mpfs_training_write_calib_retry(struct mpfs_ddr_priv_s *priv)
{
  memset(&calib_data, 0, sizeof(calib_data));

  /* Try the next write latency value */

  priv->write_latency++;
  if (priv->write_latency > WR_LATENCY_MAX)
    {
      priv->write_latency = WR_LATENCY_MIN;
      merr("Write calib fail!\n");
      return -EIO;
    }
  else
    {
      putreg32(priv->write_latency,
               MPFS_DDR_CSR_APB_CFG_DFI_T_PHY_WRLAT);
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_training_full_mtc_test
 *
 * Description:
 *   Performs a comprehensive memory test.
 *
 * Returned Value:
 *   Zero is returned on success. -EIO on error.
 *
 ****************************************************************************/

static int mpfs_training_full_mtc_test(void)
{
  uint32_t error = 0;
  uint8_t mask;

  if (mpfs_get_num_lanes() <= 3)
    {
      mask = 0x3;
    }
  else
    {
      mask = 0xf;
    }

  /* Read once to flush MTC. During write calibration the first MTC read
   * must be discarded as it is unreliable after a series of bad writes.
   */

  mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_COUNTING_PATTERN,
                MTC_ADD_SEQUENTIAL);

  /* Read using different patterns */

  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_COUNTING_PATTERN,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_WALKING_ONE,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC,
                         MTC_NO_REPEATING_PSEUDO_RANDOM, MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_ALT_ONES_ZEROS,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_ALT_5_A,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM_16BIT,
                         MTC_ADD_SEQUENTIAL);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM_8BIT,
                         MTC_ADD_SEQUENTIAL);

  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_COUNTING_PATTERN,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_WALKING_ONE,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC,
                         MTC_NO_REPEATING_PSEUDO_RANDOM, MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_ALT_ONES_ZEROS,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_ALT_5_A,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM_16BIT,
                         MTC_ADD_RANDOM);
  error |= mpfs_mtc_test(mask, 0x00, ONE_MB_MTC, MTC_PSEUDO_RANDOM_8BIT,
                         MTC_ADD_RANDOM);

  if (error)
    {
      merr("Full MTC fail\n");
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_ddr_test_32bit_nc
 *
 * Description:
 *   High level function for calling mpfs_ddr_read_write_fn(). Using
 *   non-cached memory alias from 0xc0000000.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero is returned on success. Nonzero indicates a failure.
 *
 ****************************************************************************/

static int mpfs_ddr_test_32bit_nc(struct mpfs_ddr_priv_s *priv)
{
  uint32_t error;

  /* Write and read back test from drr, non-cached access */

  error = mpfs_ddr_read_write_fn(priv,
                                 (uint64_t *)LIBERO_SETTING_DDR_32_NON_CACHE,
                                 SW_CFG_NUM_READS_WRITES,
                                 SW_CONFIG_PATTERN);

  if (error)
    {
      merr("Found errors, memory is corrupt!\n");
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mpfs_ddr_setup
 *
 * Description:
 *   This is the high-level state machine that goes through the training
 *   sequence step-by-step. DDR_TRAINING_IP_* defines are here to provide
 *   references for the manufacturer's reference HSS bootloader code. This
 *   helps when applying future patches.
 *
 * Input Parameters:
 *   priv    - Instance of the ddr private state structure
 *
 * Returned Value:
 *   Zero is returned on success. Nonzero indicates a failure.
 *
 ****************************************************************************/

static int mpfs_ddr_setup(struct mpfs_ddr_priv_s *priv)
{
  int retval;

  mpfs_ddr_sm_init(priv);

  /* DDR_TRAINING_SET_MODE_VS_BITS */

  retval = mpfs_set_mode_vs_bits(priv);

  if (retval)
    {
      return retval;
    }

  mpfs_bclksclk_sw(priv);

  /* DDR_MANUAL_ADDCMD_TRAINING_SW */

  mpfs_ddr_manual_addcmd_training(priv);
  mpfs_training_start(priv);

  /* DDR_TRAINING_IP_SM_START_CHECK */

  retval = mpfs_training_start_check(priv);

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_IP_SM_BCLKSCLK */

  retval = mpfs_training_bclksclk(priv);

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_IP_SM_ADDCMD */

  retval = mpfs_training_addcmd();

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_IP_SM_WRLVL */

  retval = mpfs_training_wrlvl_wait();

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_IP_SM_RDGATE */

  retval = mpfs_training_rdgate();

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_IP_SM_DQ_DQS */

  retval = mpfs_dq_dqs();

  if (retval)
    {
      return retval;
    }

  retval = mpfs_training_verify();

  if (retval)
    {
      return retval;
    }

  putreg32(LIBERO_SETTING_DDRPHY_MODE, MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE);

  /* DDR_TRAINING_WRITE_CALIBRATION */

  do
    {
      retval = mpfs_training_write_calibration(priv);
      if (retval == -EAGAIN)
        {
          /* On success process is continued (0 returned) */

          retval |= mpfs_training_write_calib_retry(priv);
        }
    }
  while (retval == -EAGAIN);

  /* DDR_FULL_MTC_CHECK */

  retval = mpfs_training_full_mtc_test();

  if (retval)
    {
      return retval;
    }

  /* DDR_FULL_32BIT_NC_CHECK */

  retval = mpfs_ddr_test_32bit_nc(priv);

  if (retval)
    {
      return retval;
    }

  /* DDR_TRAINING_FINISHED */

  /* Configure Segments, address mapping, CFG0/CFG1 */

  mpfs_setup_ddr_segments(LIBERO_SEG_SETUP);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_ddr_init
 *
 * Description:
 *   This is the high-level call for initializing the DDR memory.
 *
 * Returned Value:
 *   Zero is returned on success, or a negated errno on failure.
 *
 ****************************************************************************/

int mpfs_ddr_init(void)
{
  struct mpfs_ddr_priv_s *priv = &g_mpfs_ddr_priv;
  int ddr_status;

  /* On -EAGAIN, the whole training is restarted from the very beginning */

  do
    {
      ddr_status = mpfs_ddr_setup(priv);
      if (ddr_status == -EAGAIN)
        {
          mpfs_ddr_fail(priv);
        }
    }
  while (ddr_status == -EAGAIN);

  if (ddr_status == 0)
    {
      minfo("DDR setup successfully\n");
    }
  else
    {
      minfo("DDR setup returned error: %d\n", ddr_status);
      mpfs_ddr_fail(priv);
    }

  minfo("Done ddr setup\n");

  return ddr_status;
}
