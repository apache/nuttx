/****************************************************************************
 * drivers/coresight/coresight_etm3.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <debug.h>
#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>

#include <nuttx/coresight/coresight_etm.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device registers:
 * 0x000 - 0x2FC: Trace         registers
 * 0x300 - 0x314: Management    registers
 * 0x318 - 0xEFC: Trace         registers
 *
 * Coresight registers
 * 0xF00 - 0xF9C: Management    registers
 * 0xFA0 - 0xFA4: Management    registers in PFTv1.0
 *                Trace         registers in PFTv1.1
 * 0xFA8 - 0xFFC: Management    registers
 */

#define ETM_CR                   0x000
#define ETM_CCR                  0x004
#define ETM_TRIGGER              0x008
#define ETM_SR                   0x010
#define ETM_SCR                  0x014
#define ETM_TSSCR                0x018
#define ETM_TECR2                0x01c
#define ETM_TEEVR                0x020
#define ETM_TECR1                0x024
#define ETM_FFLR                 0x02c
#define ETM_ACVR(n)              (0x040 + (n * 4))
#define ETM_ACTR(n)              (0x080 + (n * 4))
#define ETM_CNTRLDVR(n)          (0x140 + (n * 4))
#define ETM_CNTENR(n)            (0x150 + (n * 4))
#define ETM_CNTRLDEVR(n)         (0x160 + (n * 4))
#define ETM_CNTVR(n)             (0x170 + (n * 4))
#define ETM_SQ12EVR              0x180
#define ETM_SQ21EVR              0x184
#define ETM_SQ23EVR              0x188
#define ETM_SQ31EVR              0x18c
#define ETM_SQ32EVR              0x190
#define ETM_SQ13EVR              0x194
#define ETM_SQR                  0x19c
#define ETM_EXTOUTEVR(n)         (0x1a0 + (n * 4))
#define ETM_CIDCVR(n)            (0x1b0 + (n * 4))
#define ETM_CIDCMR               0x1bc
#define ETM_IMPSPEC0             0x1c0
#define ETM_IMPSPEC1             0x1c4
#define ETM_IMPSPEC2             0x1c8
#define ETM_IMPSPEC3             0x1cc
#define ETM_IMPSPEC4             0x1d0
#define ETM_IMPSPEC5             0x1d4
#define ETM_IMPSPEC6             0x1d8
#define ETM_IMPSPEC7             0x1dc
#define ETM_SYNCFR               0x1e0
#define ETM_IDR                  0x1e4
#define ETM_CCER                 0x1e8
#define ETM_EXTINSELR            0x1ec
#define ETM_TESSEICR             0x1f0
#define ETM_EIBCR                0x1f4
#define ETM_TSEVR                0x1f8
#define ETM_AUXCR                0x1fc
#define ETM_TRACEIDR             0x200
#define ETM_VMIDCVR              0x240

/* Management registers (0x300-0x314) */

#define ETM_OSLAR                0x300
#define ETM_OSLSR                0x304
#define ETM_OSSRR                0x308
#define ETM_PDCR                 0x310
#define ETM_PDSR                 0x314

/* Register definition */

/* ETMCR - 0x00 */

#define ETM_CR_PWD_DWN           BIT(0)
#define ETM_CR_STALL_MODE        BIT(7)
#define ETM_CR_BRANCH_BROADCAST  BIT(8)
#define ETM_CR_ETM_PRG           BIT(10)
#define ETM_CR_ETM_EN            BIT(11)
#define ETM_CR_CYC_ACC           BIT(12)
#define ETM_CR_CTXID_SIZE        (BIT(14) | BIT(15))
#define ETM_CR_TIMESTAMP_EN      BIT(28)

/* ETM_CR_RETURN_STACK is supported by PTM, ETM not support it */

#define ETM_CR_RETURN_STACK      BIT(29)

/* ETMCCR - 0x04 */

#define ETM_CCR_FIFOFULL         BIT(23)

/* ETMPDCR - 0x310 */

#define ETM_PDCR_PWD_UP          BIT(3)

/* ETMTECR1 - 0x024 */

#define ETM_TECR1_ADDR_COMP_1    BIT(0)
#define ETM_TECR1_INC_EXC        BIT(24)
#define ETM_TECR1_START_STOP     BIT(25)

/* ETMCCER - 0x1E8 */

#define ETM_CCER_TIMESTAMP       BIT(22)
#define ETM_CCER_RETSTACK        BIT(23)

#define ETM_MODE_EXCLUDE         BIT(0)
#define ETM_MODE_CYCACC          BIT(1)
#define ETM_MODE_STALL           BIT(2)
#define ETM_MODE_TIMESTAMP       BIT(3)
#define ETM_MODE_CTXID           BIT(4)
#define ETM_MODE_BBROAD          BIT(5)
#define ETM_MODE_RET_STACK       BIT(6)
#define ETM_MODE_EXCL_KERN       BIT(30)
#define ETM_MODE_EXCL_USER       BIT(31)
#define ETM_MODE_ALL                                                    \
        (ETM_MODE_EXCLUDE | ETM_MODE_CYCACC | ETM_MODE_STALL |          \
         ETM_MODE_TIMESTAMP | ETM_MODE_CTXID | ETM_MODE_BBROAD |        \
         ETM_MODE_RET_STACK | ETM_MODE_EXCL_KERN | ETM_MODE_EXCL_USER)

#define ETM_SQR_MASK             0x3
#define ETM_TRACEID_MASK         0x3f
#define ETM_EVENT_MASK           0x1ffff
#define ETM_SYNC_MASK            0xfff
#define ETM_ALL_MASK             0xffffffff

#define ETM_SR_PROGRAM           BIT(1)
#define ETM_SEQ_STATE_MAX_VAL    0x2
#define ETM_PORT_SIZE_MASK       (GENMASK(21, 21) | GENMASK(6, 4))

#define ETM_ARCH_V3_3            0x23
#define ETM_ARCH_V3_5            0x25
#define PFT_ARCH_V1_0            0x30
#define PFT_ARCH_V1_1            0x31

/* Hard wired, always true */

#define ETM_HARD_WIRE_RES_A      ((0x0f << 0) | (0x06 << 4))

/* Single addr comparator 1 */

#define ETM_ADD_COMP_0           ((0x00 << 7) | (0x00 << 11))

/* NOT(A) */

#define ETM_EVENT_NOT_A          BIT(14)

#define ETM_DEFAULT_EVENT_VAL \
        (ETM_HARD_WIRE_RES_A | ETM_ADD_COMP_0 | ETM_EVENT_NOT_A)

#define ETM3X_SUPPORTED_OPTIONS \
        (ETM_CR_CYC_ACC | ETM_CR_TIMESTAMP_EN | ETM_CR_RETURN_STACK)

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int etm_enable(FAR struct coresight_dev_s *csdev);
static void etm_disable(FAR struct coresight_dev_s *csdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_source_ops_s g_etm_source_ops =
{
  .enable  = etm_enable,
  .disable = etm_disable,
};

static const struct coresight_ops_s g_etm_ops =
{
  .source_ops = &g_etm_source_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_CORESIGHT_ETM_USE_COPROCESSOR
static inline void etm_write_reg(FAR struct coresight_etm_dev_s *etmdev,
                                 uint32_t val, uint32_t off)
{
  coresight_put32(val, etmdev->csdev.addr + off);
}

static inline uint32_t etm_read_reg(FAR struct coresight_etm_dev_s *etmdev,
                                    uint32_t off)
{
  return coresight_get32(etmdev->csdev.addr + off);
}
#endif

/****************************************************************************
 * Name: etm_modify_reg32
 *
 * Description:
 *   Set a bitmask of register to specific value.
 *
 ****************************************************************************/

static void etm_modify_reg32(FAR struct coresight_etm_dev_s *etmdev,
                             uint32_t val, uint32_t mask, uint32_t off)
{
  uint32_t temp = etm_read_reg(etmdev, off);

  etm_write_reg(etmdev, (temp & ~mask) | (val & mask), off);
}

/****************************************************************************
 * Name: etm_os_unlock
 *
 * Description:
 *   When the ETM trace registers are locked, any attempt to access the
 *   locked registers returns a slave-generated error response.In ETMv3.5,
 *   the OS Lock is always set from an ETM reset. ARM recommends that, after
 *   programming the ETM registers, you always execute an ISB instruction to
 *   ensure that all updates are committed to the ETM before you restart
 *   normal code execution.
 *
 ****************************************************************************/

static void etm_os_unlock(FAR struct coresight_etm_dev_s *etmdev)
{
  etm_write_reg(etmdev, 0x0, ETM_OSLAR);
}

/****************************************************************************
 * Name: etm_clr_pwrdwn
 *
 * Description:
 *   When pwrdown bit is set to 1, the ETM must be powered down and disabled,
 *   and then operated in a low power mode with all clocks stopped. When this
 *   bit is set to 1, writes to some registers and fields might be ignored.
 *   ARM recommends that you use a read-modify-write procedure when modifying
 *   the ETMCR.
 *
 ****************************************************************************/

static void etm_clr_pwrdwn(FAR struct coresight_etm_dev_s *etmdev)
{
  etm_modify_reg32(etmdev, 0, ETM_CR_PWD_DWN, ETM_CR);
}

/****************************************************************************
 * Name: etm_set_pwrdwn
 ****************************************************************************/

static void etm_set_pwrdwn(FAR struct coresight_etm_dev_s *etmdev)
{
  etm_modify_reg32(etmdev, ETM_CR_PWD_DWN, ETM_CR_PWD_DWN, ETM_CR);
}

/****************************************************************************
 * Name: etm_set_pwrup
 *
 * Description:
 *   Power is provided to the ETM trace registers. This register can only be
 *   accessed using a memory-mapped interface or from an external debugger.
 *
 ****************************************************************************/

static void etm_set_pwrup(FAR struct coresight_etm_dev_s *etmdev)
{
  coresight_modify32(ETM_PDCR_PWD_UP, ETM_PDCR_PWD_UP,
                     etmdev->csdev.addr + ETM_PDCR);
}

/****************************************************************************
 * Name: etm_clr_pwrup
 ****************************************************************************/

static void etm_clr_pwrup(FAR struct coresight_etm_dev_s *etmdev)
{
  coresight_modify32(0, ETM_PDCR_PWD_UP, etmdev->csdev.addr + ETM_PDCR);
}

/****************************************************************************
 * Name: etm_timeout
 *
 * Description:
 *   Loop until a bitmask of register has changed to a specific value.
 *
 ****************************************************************************/

static int etm_timeout(FAR struct coresight_etm_dev_s *etmdev,
                       uint32_t val, uint32_t mask, uint32_t off)
{
  int i;

  for (i = CONFIG_CORESIGHT_TIMEOUT; i > 0; i--)
    {
      uint32_t value = etm_read_reg(etmdev, off);
      if ((value & mask) == val)
        {
          return 0;
        }

      up_udelay(1);
    }

  return -EAGAIN;
}

/****************************************************************************
 * Name: etm_set_program
 *
 * Description:
 *   Set ETM Programming bit to disable all operations during programming.
 *   When programming the ETM registers you must enable all the changes at
 *   the same time.
 *
 ****************************************************************************/

static void etm_set_program(FAR struct coresight_etm_dev_s *etmdev)
{
  etm_modify_reg32(etmdev, ETM_CR_ETM_PRG, ETM_CR_ETM_PRG, ETM_CR);

  if (etm_timeout(etmdev, ETM_SR_PROGRAM, ETM_SR_PROGRAM, ETM_SR) < 0)
    {
      cserr("timeout observed at setting ETM_SR_PROGRAM\n");
    }
}

/****************************************************************************
 * Name: etm_clr_program
 ****************************************************************************/

static void etm_clr_program(FAR struct coresight_etm_dev_s *etmdev)
{
  etm_modify_reg32(etmdev, 0, ETM_CR_ETM_PRG, ETM_CR);

  if (etm_timeout(etmdev, 0, ETM_SR_PROGRAM, ETM_SR) < 0)
    {
      cserr("timeout observed at clearing ETM_SR_PROGRAM\n");
    }
}

/****************************************************************************
 * Name: etm_init_arch_data
 *
 * Description:
 *   Get capabilities of current ETM architecture version.
 *
 ****************************************************************************/

static void etm_init_arch_data(FAR struct coresight_etm_dev_s *etmdev)
{
  uint32_t etmccr;

  coresight_unlock(etmdev->csdev.addr);
  etm_os_unlock(etmdev);
  etm_clr_pwrdwn(etmdev);
  etm_set_pwrup(etmdev);
  etm_set_program(etmdev);

  etmdev->arch = BMVAL(etm_read_reg(etmdev, ETM_IDR), 4, 11);
  etmdev->port_size = etm_read_reg(etmdev, ETM_CR) & ETM_PORT_SIZE_MASK;

  etmccr = etm_read_reg(etmdev, ETM_CCR);
  etmdev->nr_addr_cmp = BMVAL(etmccr, 0, 3) * 2;
  etmdev->nr_cntr = BMVAL(etmccr, 13, 15);
  etmdev->nr_ext_inp = BMVAL(etmccr, 17, 19);
  etmdev->nr_ext_out = BMVAL(etmccr, 20, 22);
  etmdev->nr_ctxid_cmp = BMVAL(etmccr, 24, 25);

  etm_clr_program(etmdev);
  etm_clr_pwrup(etmdev);
  etm_set_pwrdwn(etmdev);
  coresight_lock(etmdev->csdev.addr);
}

/****************************************************************************
 * Name: etm_arch_supported
 *
 * Description:
 *   If ETM/PTM implement is supported by current driver.
 *
 ****************************************************************************/

static bool etm_arch_supported(uint8_t arch)
{
  switch (arch)
    {
      case ETM_ARCH_V3_3:
      case ETM_ARCH_V3_5:
      case PFT_ARCH_V1_0:
      case PFT_ARCH_V1_1:
        return true;
      default:
        return false;
    }
}

/****************************************************************************
 * Name: etm_set_default
 *
 * Description:
 *   Setup default ETM config.
 *
 ****************************************************************************/

static void etm_set_default(struct etm_config_s *config)
{
  int i;

  /* Trace all memory, set according to spec */

  config->enable_ctrl1 = BIT(24);
  config->enable_ctrl2 = 0x0;
  config->enable_event = ETM_HARD_WIRE_RES_A;

  /* Disable all other event */

  config->trigger_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_12_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_21_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_23_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_31_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_32_event = ETM_DEFAULT_EVENT_VAL;
  config->seq_13_event = ETM_DEFAULT_EVENT_VAL;
  config->timestamp_event = ETM_DEFAULT_EVENT_VAL;

  for (i = 0; i < ETM_MAX_CNTR; i++)
    {
      config->cntr_event[i] = ETM_DEFAULT_EVENT_VAL;
      config->cntr_rld_event[i] = ETM_DEFAULT_EVENT_VAL;
    }

  config->sync_freq = 0x400;
}

/****************************************************************************
 * Name: etm_hw_enable
 ****************************************************************************/

static void etm_hw_enable(FAR struct coresight_etm_dev_s *etmdev)
{
  FAR struct etm_config_s *config = &etmdev->cfg;
  uint32_t etmcr;
  int i;

  coresight_unlock(etmdev->csdev.addr);
  etm_os_unlock(etmdev);
  etm_set_pwrup(etmdev);
  etm_clr_pwrdwn(etmdev);
  etm_set_program(etmdev);

  etmcr = etm_read_reg(etmdev, ETM_CR);
  etmcr &= ~ETM3X_SUPPORTED_OPTIONS;
  etmcr |= ETM_CR_ETM_EN;
  etm_write_reg(etmdev, config->ctrl | etmcr, ETM_CR);
  etm_write_reg(etmdev, config->trigger_event, ETM_TRIGGER);
  etm_write_reg(etmdev, config->startstop_ctrl, ETM_TSSCR);
  etm_write_reg(etmdev, config->enable_event, ETM_TEEVR);
  etm_write_reg(etmdev, config->enable_ctrl1, ETM_TECR1);
  etm_write_reg(etmdev, config->fifofull_level, ETM_FFLR);
  for (i = 0; i < etmdev->nr_addr_cmp; i++)
    {
      etm_write_reg(etmdev, config->addr_val[i], ETM_ACVR(i));
      etm_write_reg(etmdev, config->addr_acctype[i], ETM_ACTR(i));
    }

  for (i = 0; i < etmdev->nr_cntr; i++)
    {
      etm_write_reg(etmdev, config->cntr_rld_val[i], ETM_CNTRLDVR(i));
      etm_write_reg(etmdev, config->cntr_event[i], ETM_CNTENR(i));
      etm_write_reg(etmdev, config->cntr_rld_event[i], ETM_CNTRLDEVR(i));
      etm_write_reg(etmdev, config->cntr_val[i], ETM_CNTVR(i));
    }

  etm_write_reg(etmdev, config->seq_12_event, ETM_SQ12EVR);
  etm_write_reg(etmdev, config->seq_21_event, ETM_SQ21EVR);
  etm_write_reg(etmdev, config->seq_23_event, ETM_SQ23EVR);
  etm_write_reg(etmdev, config->seq_31_event, ETM_SQ31EVR);
  etm_write_reg(etmdev, config->seq_32_event, ETM_SQ32EVR);
  etm_write_reg(etmdev, config->seq_13_event, ETM_SQ13EVR);
  etm_write_reg(etmdev, config->seq_curr_state, ETM_SQR);
  for (i = 0; i < etmdev->nr_ext_out; i++)
    {
      etm_write_reg(etmdev, ETM_DEFAULT_EVENT_VAL, ETM_EXTOUTEVR(i));
    }

  for (i = 0; i < etmdev->nr_ctxid_cmp; i++)
    {
      etm_write_reg(etmdev, config->ctxid_pid[i], ETM_CIDCVR(i));
    }

  etm_write_reg(etmdev, config->ctxid_mask, ETM_CIDCMR);
  etm_write_reg(etmdev, config->sync_freq, ETM_SYNCFR);

  /* No external input selected */

  etm_write_reg(etmdev, 0x0, ETM_EXTINSELR);
  etm_write_reg(etmdev, config->timestamp_event, ETM_TSEVR);

  /* No auxiliary control selected */

  etm_write_reg(etmdev, 0x0, ETM_AUXCR);

  etm_write_reg(etmdev, etmdev->traceid, ETM_TRACEIDR);

  /* No VMID comparator value selected */

  etm_write_reg(etmdev, 0x0, ETM_VMIDCVR);

  etm_clr_program(etmdev);
  coresight_lock(etmdev->csdev.addr);
}

/****************************************************************************
 * Name: etm_hw_disable
 ****************************************************************************/

static void etm_hw_disable(FAR struct coresight_etm_dev_s *etmdev)
{
  FAR struct etm_config_s *config = &etmdev->cfg;
  int i;

  coresight_unlock(etmdev->csdev.addr);
  etm_set_program(etmdev);

  /* Read back sequencer and counters for post trace analysis */

  config->seq_curr_state = etm_read_reg(etmdev, ETM_SQR) & ETM_SQR_MASK;
  for (i = 0; i < etmdev->nr_cntr; i++)
    {
      config->cntr_val[i] = etm_read_reg(etmdev, ETM_CNTVR(i));
    }

  etm_set_pwrdwn(etmdev);
  coresight_lock(etmdev->csdev.addr);
}

/****************************************************************************
 * Name: etm_enable
 ****************************************************************************/

static int etm_enable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_etm_dev_s *etmdev =
    (FAR struct coresight_etm_dev_s *)csdev;
  int ret;

  ret = coresight_claim_device(etmdev->csdev.addr);
  if (ret < 0)
    {
      return ret;
    }

  etm_hw_enable(etmdev);
  return ret;
}

/****************************************************************************
 * Name: etm_enable
 ****************************************************************************/

static void etm_disable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_etm_dev_s *etmdev =
    (FAR struct coresight_etm_dev_s *)csdev;

  etm_hw_disable(etmdev);
  coresight_disclaim_device(etmdev->csdev.addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etm_config
 *
 * Description:
 *   Configure the etm device.
 *
 * Input Parameters:
 *   etmdev  - Pointer to the ETM device to config.
 *   config  - Configuration need to be set to ETM device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int etm_config(FAR struct coresight_etm_dev_s *etmdev,
               FAR const struct etm_config_s *config)
{
  coresight_unlock(etmdev->csdev.addr);
  etm_os_unlock(etmdev);
  etm_clr_pwrdwn(etmdev);
  etm_set_pwrup(etmdev);
  etm_set_program(etmdev);

  memcpy(&etmdev->cfg, config, sizeof(struct etm_config_s));

  if ((etmdev->cfg.ctrl & ETM_CR_RETURN_STACK) &&
      !(etm_read_reg(etmdev, ETM_CCER) & ETM_CCER_RETSTACK))
    {
      etmdev->cfg.ctrl &= ~ETM_CR_RETURN_STACK;
    }

  etm_clr_program(etmdev);
  etm_clr_pwrup(etmdev);
  etm_set_pwrdwn(etmdev);
  coresight_lock(etmdev->csdev.addr);

  return 0;
}

/****************************************************************************
 * Name: etm_register
 *
 * Description:
 *   Register an ETM/PTM devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to an ETM device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_etm_dev_s *
etm_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_etm_dev_s *etmdev;
  FAR struct coresight_dev_s *csdev;
  int ret;

  etmdev = kmm_zalloc(sizeof(struct coresight_etm_dev_s));
  if (etmdev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  etmdev->cpu = desc->cpu;
  etmdev->csdev.addr = desc->addr;
  etm_init_arch_data(etmdev);

  if (!etm_arch_supported(etmdev->arch))
    {
      kmm_free(etmdev);
      cserr("%s:current implement version is not supported\n", desc->name);
      return NULL;
    }

  etmdev->traceid = coresight_get_cpu_trace_id(etmdev->cpu);
  etm_set_default(&etmdev->cfg);

  csdev = &etmdev->csdev;
  csdev->ops = &g_etm_ops;
  ret = coresight_register(csdev, desc);
  if (ret < 0)
    {
      kmm_free(etmdev);
      cserr("%s:register failed\n", desc->name);
      return NULL;
    }

  return etmdev;
}

/****************************************************************************
 * Name: etm_unregister
 *
 * Description:
 *   Unregister an EMT/PTM device.
 *
 ****************************************************************************/

void etm_unregister(FAR struct coresight_etm_dev_s *etmdev)
{
  coresight_unregister(&etmdev->csdev);
  kmm_free(etmdev);
}
