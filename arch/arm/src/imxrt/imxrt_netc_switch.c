/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_switch.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <string.h>
#include <debug.h>
#include <endian.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev.h>

#include "imxrt_netc_priv.h"
#include "hardware/imxrt_netc.h"
#include "hardware/imxrt_netc_switch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SWREG(r)      (IMXRT_SW0_BASE + (r))
#define PORTREG(p, r) (IMXRT_NETC_SW_PORT(p) + (r))
#define SWITCH_NCMD  8

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum imxrt_netc_switch_state_e
{
  SWITCH_UNINITIALIZED = 0,
  SWITCH_READY,
  SWITCH_FAILED
};

struct imxrt_netc_switch_dma_s
{
  struct imxrt_netc_cmd_s command[SWITCH_NCMD];
  uint32_t data[NETC_DMA_ALIGN / sizeof(uint32_t)]
    aligned_data(NETC_DMA_ALIGN);
};

struct imxrt_netc_switch_s
{
  const struct imxrt_netc_hw_s *hw;
  const struct imxrt_netc_enetc_s *config;
  struct imxrt_netc_switch_dma_s *dma_cpu;
  uint32_t dma_bus;
  struct imxrt_netc_link_s link[4];
  enum imxrt_netc_switch_state_e state;
  unsigned int producer;
  uint8_t fdbmac[6];              /* Installed address may differ from config */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Switch operations and initialization use the network lock. PHYLIB polls
 * independently and serializes PHY register access with the MDIO bus lock.
 */

static struct imxrt_netc_switch_s g_switch;

static_assert(sizeof(struct imxrt_netc_cmd_s) == 32,
              "NETC NTMP descriptor size");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_netc_fdb(const uint8_t *mac, bool add)
{
  struct imxrt_netc_cmd_s *cmd;
  uint32_t *data = g_switch.dma_cpu->data;
  unsigned int next = (g_switch.producer + 1) % SWITCH_NCMD;
  uint32_t status;
  int ret;

  memset(data, 0, NETC_DMA_ALIGN);
  data[0] = htole32(add ? NETC_NTMP_FDB_CFGEU : 0);
  memcpy(&data[1], mac, 6);
  data[9] = htole32(UINT32_C(1) << g_switch.hw->sw_cpu_port_reg_idx);
  data[11] = htole32(UINT32_MAX);
  up_clean_dcache((uintptr_t)data, (uintptr_t)data + NETC_DMA_ALIGN);

  cmd = &g_switch.dma_cpu->command[g_switch.producer];
  memset(cmd, 0, sizeof(*cmd));
  cmd->address = htole32(g_switch.dma_bus +
                          offsetof(struct imxrt_netc_switch_dma_s, data));
  cmd->lengths = htole32(((add ? NETC_NTMP_FDB_REQUEST :
                          NETC_NTMP_FDB_DELETE) << 20) |
                        NETC_NTMP_FDB_RESPONSE);
  cmd->command = htole32(NETC_NTMP_VERSION2 | (NETC_NTMP_FDB << 16) |
                        NETC_NTMP_EXACT | (add ?
                        NETC_NTMP_ADDQUERYUPDATE : NETC_NTMP_DELETE));
  cmd->format = htole32(NETC_NTMP_NPF);
  up_clean_dcache((uintptr_t)cmd, (uintptr_t)cmd + sizeof(*cmd));
  UP_DSB();
  putreg32(next, SWREG(NETC_SW_CBDRPIR));

  ret = imxrt_netc_wait(SWREG(NETC_SW_CBDRCIR), NETC_BDR_INDEX_MASK, next);
  if (ret < 0)
    {
      /* The command and response storage remain owned by DMA until FLR. */

      g_switch.state = SWITCH_FAILED;
      nerr("NETC switch command timed out\n");
      return ret;
    }

  UP_DMB();
  up_invalidate_dcache((uintptr_t)cmd, (uintptr_t)cmd + sizeof(*cmd));
  up_invalidate_dcache((uintptr_t)data, (uintptr_t)data + NETC_DMA_ALIGN);
  g_switch.producer = next;
  status = le32toh(cmd->command);
  if ((status & NETC_NTMP_READY) == 0 ||
      (status & NETC_NTMP_ERROR_MASK) != 0)
    {
      nerr("NETC FDB command response: %08lx\n", (unsigned long)status);
      return -EIO;
    }

  /* Add/query/update reports 0 for a new entry and 1 for
   * an existing entry.
   */

  if (add && (status & UINT16_MAX) > 1)
    {
      nerr("NETC FDB update returned invalid match count %lu\n",
           (unsigned long)(status & UINT16_MAX));
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_switch_uninitialize(void)
{
  unsigned int port;
  int ret;

  if (g_switch.state == SWITCH_UNINITIALIZED)
    {
      return OK;
    }

  for (port = 0; port < g_switch.hw->portcount; port++)
    {
      imxrt_netc_port_uninitialize(&g_switch.hw->ports[port],
                                   &g_switch.link[port]);
      modifyreg32(PORTREG(g_switch.hw->ports[port].reg_idx, NETC_PORT_POR),
                  0, NETC_PORT_DISABLE);
    }

  modifyreg32(PORTREG(g_switch.hw->sw_cpu_port_reg_idx, NETC_PORT_POR),
              0, NETC_PORT_DISABLE);

  ret = imxrt_netc_reset(IMXRT_NETC_SW0_PF, IMXRT_SW0_BASE);
  if (ret < 0)
    {
      g_switch.state = SWITCH_FAILED;
      return ret;
    }

  putreg16(0, IMXRT_NETC_SW0_PF + NETC_PCI_CMD);
  kmm_free(g_switch.dma_cpu);
  memset(&g_switch, 0, sizeof(g_switch));
  return OK;
}

int imxrt_netc_switch_initialize(const struct imxrt_netc_hw_s *hw,
                                const struct imxrt_netc_enetc_s *config,
                                uint32_t clock_hz)
{
  unsigned int port;
  uint32_t portmask = 0;
  uint32_t status;
  unsigned int tc;
  int ret;

  if (g_switch.state != SWITCH_UNINITIALIZED ||
      (getreg16(IMXRT_NETC_SW0_PF + NETC_PCI_CMD) &
       NETC_PCI_CMD_ENABLE) != 0)
    {
      return -EBUSY;
    }

  for (port = 0; port < hw->portcount; port++)
    {
      if (!config->ports[port].enabled)
        {
          continue;
        }

      ret = imxrt_netc_port_prepare(&hw->ports[port], &config->ports[port]);
      if (ret < 0)
        {
          nerr("NETC switch port %u preparation failed: %d\n", port, ret);
          return ret;
        }
    }

  g_switch.dma_cpu = imxrt_netc_alloc(sizeof(*g_switch.dma_cpu),
                                     &g_switch.dma_bus);
  if (g_switch.dma_cpu == NULL)
    {
      return -ENOMEM;
    }

  g_switch.hw = hw;
  g_switch.config = config;
  for (port = 0; port < hw->portcount; port++)
    {
      if (config->ports[port].enabled)
        {
          portmask |= UINT32_C(1) << hw->ports[port].reg_idx;
        }
    }

  g_switch.state = SWITCH_FAILED;
  putreg16(NETC_PCI_CMD_ENABLE, IMXRT_NETC_SW0_PF + NETC_PCI_CMD);
  ret = imxrt_netc_wait(SWREG(NETC_COMMON_OSR),
                        NETC_COMMON_STATE, 0);
  if (ret < 0)
    {
      return ret;
    }

  status = getreg32(SWREG(NETC_COMMON_OSR));
  if ((status & NETC_COMMON_ITM_STATE) != 0)
    {
      nerr("NETC switch index table memory overprovisioned: OSR=%08lx\n",
           (unsigned long)status);
      return -EIO;
    }

  for (port = 0; port < hw->portcount; port++)
    {
      modifyreg32(PORTREG(hw->ports[port].reg_idx, NETC_PORT_POR),
                  0, NETC_PORT_DISABLE);
    }

  modifyreg32(PORTREG(hw->sw_cpu_port_reg_idx, NETC_PORT_POR),
              0, NETC_PORT_DISABLE);

  putreg32(portmask, SWREG(NETC_SW_VFHTDECR0));
  putreg32(NETC_SW_SHARED_FID, SWREG(NETC_SW_VFHTDECR1));
  putreg32(NETC_SW_LEARN_FLOOD, SWREG(NETC_SW_VFHTDECR2));
  putreg32(UINT16_MAX, SWREG(NETC_SW_FDBHTMCR));

  int reg_idx = 0;

  for (port = 0; port < hw->portcount + 1; port++)
    {
      if (port < hw->portcount)
        {
          reg_idx = hw->ports[port].reg_idx;

          if (!config->ports[port].enabled)
            {
              putreg32(0, PORTREG(reg_idx, NETC_SW_BPSTGSR));
              continue;
            }
        }
      else
        {
          reg_idx = hw->sw_cpu_port_reg_idx;
        }

      putreg32(NETC_SW_STG_FORWARD, PORTREG(reg_idx, NETC_SW_BPSTGSR));
      putreg32(NETC_SW_DEFAULT_VLAN, PORTREG(reg_idx, NETC_SW_BPDVR));
      putreg32(UINT16_MAX, PORTREG(reg_idx, NETC_SW_BPCR));
      putreg32(NETC_SW_STANDARD_TPIDS, PORTREG(reg_idx, NETC_SW_PTAR));
      putreg32(NETC_SW_NULL_STREAM, PORTREG(reg_idx, NETC_SW_PISIDCR));
      putreg32(0, PORTREG(reg_idx, NETC_SW_PIPFCR));

      /* ERR051649: only VLAN-to-QoS profile 0 is supported. Keep
       * IPV2ICMPMR0 at reset (single ICM priority), per ERR051444.
       */

      putreg32(0, PORTREG(reg_idx, NETC_SW_PQOSMR));
      putreg32(0, PORTREG(reg_idx, NETC_SW_PIPV2QMR0));
      putreg32(0xff, PORTREG(reg_idx, NETC_SW_PDGSR));
      for (tc = 0; tc < 8; tc++)
        {
          putreg32(NETC_PORT_MAXSDU_DISABLE,
                   PORTREG(reg_idx, NETC_PORT_PTCTMSDUR(tc)));
        }

      if (port < hw->portcount)
        {
          putreg32(NETC_SW_STORE_FORWARD, PORTREG(reg_idx, NETC_SW_PCTFCR));
          ret = imxrt_netc_port_initialize(&hw->ports[port],
                                           &config->ports[port],
                                           &g_switch.link[port],
                                           clock_hz);
          if (ret < 0)
            {
              nerr("NETC switch port %u initialization: %d\n", port, ret);
              return ret;
            }
        }
      else
        {
          imxrt_netc_pseudo(IMXRT_NETC_SW_PORT(reg_idx));
        }
    }

  putreg32(NETC_DMA_NONCOHERENT, SWREG(NETC_SW_CCAR));
  up_clean_dcache((uintptr_t)g_switch.dma_cpu,
                 (uintptr_t)g_switch.dma_cpu + sizeof(*g_switch.dma_cpu));
  putreg32(g_switch.dma_bus +
           offsetof(struct imxrt_netc_switch_dma_s, command),
           SWREG(NETC_SW_CBDRBAR0));
  putreg32(0, SWREG(NETC_SW_CBDRBAR1));
  putreg32(0, SWREG(NETC_SW_CBDRPIR));
  putreg32(0, SWREG(NETC_SW_CBDRCIR));
  putreg32(SWITCH_NCMD, SWREG(NETC_SW_CBDRLENR));
  UP_DSB();
  putreg32(NETC_BDR_EN, SWREG(NETC_SW_CBDRMR));
  ret = imxrt_netc_fdb(config->mac, true);
  if (ret == OK)
    {
      memcpy(g_switch.fdbmac, config->mac, sizeof(g_switch.fdbmac));
      g_switch.state = SWITCH_READY;
    }

  return ret;
}

int imxrt_netc_switch_cpu(bool enable, const uint8_t *mac)
{
  int ret;

  if (g_switch.state == SWITCH_UNINITIALIZED)
    {
      return -ENODEV;
    }

  /* Gate both static-FDB delivery and flooding before changing the CPU
   * function. External membership and learned entries remain untouched.
   */

  modifyreg32(PORTREG(g_switch.hw->sw_cpu_port_reg_idx, NETC_PORT_POR),
              0, NETC_PORT_DISABLE);
  modifyreg32(SWREG(NETC_SW_VFHTDECR0),
              UINT32_C(1) << g_switch.hw->sw_cpu_port_reg_idx, 0);
  UP_DSB();
  ret = imxrt_netc_wait(PORTREG(g_switch.hw->sw_cpu_port_reg_idx,
                                NETC_PORT_PSR), NETC_PORT_BUSY, 0);
  if (ret < 0 || !enable)
    {
      return ret;
    }

  if (g_switch.state != SWITCH_READY)
    {
      return -EIO;
    }

  if (memcmp(g_switch.fdbmac, mac, sizeof(g_switch.fdbmac)) != 0)
    {
      ret = imxrt_netc_fdb(mac, true);
      if (ret == OK)
        {
          ret = imxrt_netc_fdb(g_switch.fdbmac, false);
        }

      if (ret < 0)
        {
          return ret;
        }

      memcpy(g_switch.fdbmac, mac, sizeof(g_switch.fdbmac));
    }

  modifyreg32(SWREG(NETC_SW_VFHTDECR0), 0,
              UINT32_C(1) << g_switch.hw->sw_cpu_port_reg_idx);
  UP_DSB();
  modifyreg32(PORTREG(g_switch.hw->sw_cpu_port_reg_idx,
                      NETC_PORT_POR), NETC_PORT_DISABLE, 0);
  return OK;
}

int imxrt_netc_switch_link(uint8_t *linkmask)
{
  unsigned int port;
  uint16_t speed;
  int result = OK;
  int ret;

  *linkmask = 0;
  if (g_switch.state == SWITCH_UNINITIALIZED)
    {
      return -ENODEV;
    }

  for (port = 0; port < g_switch.hw->portcount; port++)
    {
      if (!g_switch.config->ports[port].enabled)
        {
          continue;
        }

      ret = imxrt_netc_port_link(&g_switch.link[port], &speed);
      if (ret < 0)
        {
          nerr("NETC switch port %u link: %d\n", port, ret);
          result = ret;
          speed = 0;
        }

      if (speed != g_switch.link[port].speed)
        {
          ret = imxrt_netc_port_mode(
                  IMXRT_NETC_SW_PORT(g_switch.hw->ports[port].reg_idx),
                  IMXRT_NETC_SW_MAC(g_switch.hw->ports[port].reg_idx),
                  g_switch.config->ports[port].interface,
                  speed);
          if (ret < 0)
            {
              nerr("NETC switch port %u MAC: %d\n", port, ret);
              result = ret;
              speed = 0;
            }

          g_switch.link[port].speed = speed;
          ninfo("NETC port %u: %u Mbps\n", port, speed);
        }

      if (speed != 0)
        {
          *linkmask |= 1 << port;
        }
    }

  return result;
}

#ifdef CONFIG_NETDEV_IOCTL
int imxrt_netc_switch_ioctl(int cmd, unsigned long arg)
{
  struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)arg;
  struct mii_ioctl_data_s request;
  unsigned int port;
  int ret;

  if (cmd != SIOCGMIIPHY && cmd != SIOCGMIIREG && cmd != SIOCSMIIREG)
    {
      return -ENOTTY;
    }

  if (req == NULL)
    {
      return -EINVAL;
    }

  /* phy_id[9:8] selects an external switch port, [4:0] is its MDIO
   * address. SIOCGMIIPHY preserves the requested port and resolves address.
   */

  port = req->phy_id >> 8;
  if (g_switch.state == SWITCH_UNINITIALIZED)
    {
      return -ENODEV;
    }

  if (port >= g_switch.hw->portcount || (req->phy_id & 0xe0) != 0 ||
      !g_switch.config->ports[port].enabled)
    {
      return -EINVAL;
    }

  if (cmd == SIOCGMIIPHY)
    {
      req->phy_id = (port << 8) | g_switch.link[port].phydev->mdio_addr;
      return OK;
    }

  if ((req->phy_id & 31) != g_switch.link[port].phydev->mdio_addr ||
      req->reg_num > 31)
    {
      return -EINVAL;
    }

  request = *req;
  request.phy_id &= 31;
  ret = phy_ioctl(g_switch.link[port].phydev, cmd,
                   (unsigned long)&request);
  if (ret == OK)
    {
      req->val_out = request.val_out;
    }

  return ret;
}
#endif
