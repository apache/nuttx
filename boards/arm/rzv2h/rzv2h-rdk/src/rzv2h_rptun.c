/****************************************************************************
 * boards/arm/rzv2h/rzv2h-rdk/src/rzv2h_rptun.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <sys/types.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/rptun/rptun.h>
#include <arch/board/board.h>

#include "r_mhu_b_ns.h"
#include "rzv2h_irq_id.h"
#include "rzv2h_rptun.h"
#include "rzv2h_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_RPTUN_MHU_CHANNEL       3
#define RZV2H_RPTUN_MHU_IRQ_PRIORITY  12
#define RZV2H_RPMSG_SHUTDOWN          0xef56a55a
#define RZV2H_RPTUN_RESTART_STACKSIZE 4096

#if ((1ULL << RZV2H_RPTUN_MHU_CHANNEL) & \
     BSP_FEATURE_MHU_B_NS_VALID_CHANNEL_MASK) == 0
#  error RZ/V2H CR8_0 RPMsg MHU channel is not supported by this FSP BSP
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MHU-B channel and receive IRQ must match the Linux mailbox configuration.
 * On CR8, receiving a MSG interrupt makes this endpoint the response side of
 * the MHU-B pair; msgSend() therefore kicks Linux through the paired
 * response channel.  Keep the message words in the non-cacheable shared MPU
 * window.
 */

static mhu_b_ns_instance_ctrl_t g_rzv2h_rptun_mhu_ctrl;

static const mhu_cfg_t g_rzv2h_rptun_mhu_cfg =
{
  .channel         = RZV2H_RPTUN_MHU_CHANNEL,
  .rx_ipl          = RZV2H_RPTUN_MHU_IRQ_PRIORITY,
  .rx_irq          = RZV2H_IRQ_MHU_MSG3_NS,
  .p_callback      = NULL,
  .p_shared_memory = NULL,
  .p_context       = NULL,
};

static const mhu_instance_t g_rzv2h_rptun_mhu =
{
  .p_ctrl = (mhu_ctrl_t *)&g_rzv2h_rptun_mhu_ctrl,
  .p_cfg  = &g_rzv2h_rptun_mhu_cfg,
  .p_api  = &g_mhu_b_ns_on_mhu_b_ns,
};

static struct rzv2h_rptun_config_s g_rzv2h_rptun_config =
{
  .mhu       = &g_rzv2h_rptun_mhu,

  /* FSP pairs the channel 3 MSG receiver with response transmitter 6. */

  .tx_status = &R_MHU_NS6->RSP_INT_STSn,
  .cpuname   = BOARD_RPTUN_REMOTE_CPUNAME,
  .notifyid  = BOARD_RPTUN_NOTIFYID,
};

static bool g_rzv2h_rptun_initialized;

#ifdef CONFIG_RZV2H_RPTUN_ECHO
static struct rpmsg_endpoint g_rzv2h_echo_ept;
static bool g_rzv2h_echo_created;
static bool g_rzv2h_restart_pending;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RZV2H_RPTUN_ECHO

static int rzv2h_rptun_restart(int argc, FAR char *argv[])
{
  irqstate_t flags;
  int ret;

  (void)argc;
  (void)argv;

  ret = rptun_poweroff(BOARD_RPTUN_REMOTE_CPUNAME);
  if (ret >= 0)
    {
      rzv2h_rptun_resource_reset();
      ret = rptun_boot(BOARD_RPTUN_REMOTE_CPUNAME);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: RZ/V2H RPTUN restart failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "RZ/V2H RPTUN ready for next Linux session\n");
    }

  flags = enter_critical_section();
  g_rzv2h_restart_pending = false;
  leave_critical_section(flags);
  return ret;
}

static int rzv2h_echo_ept_cb(FAR struct rpmsg_endpoint *ept,
                             FAR void *data, size_t len, uint32_t src,
                             FAR void *priv)
{
  uint32_t value;
  irqstate_t flags;
  bool restart;
  int ret;

  (void)src;
  (void)priv;

  if (len >= sizeof(value))
    {
      memcpy(&value, data, sizeof(value));
      if (value == RZV2H_RPMSG_SHUTDOWN)
        {
          syslog(LOG_INFO, "RZ/V2H RPMsg echo: shutdown received\n");

          flags = enter_critical_section();
          restart = !g_rzv2h_restart_pending;
          g_rzv2h_restart_pending = true;
          leave_critical_section(flags);

          if (restart)
            {
              ret = kthread_create("rptun-restart",
                                   SCHED_PRIORITY_DEFAULT,
                                   RZV2H_RPTUN_RESTART_STACKSIZE,
                                   rzv2h_rptun_restart, NULL);
              if (ret < 0)
                {
                  flags = enter_critical_section();
                  g_rzv2h_restart_pending = false;
                  leave_critical_section(flags);
                  syslog(LOG_ERR,
                         "ERROR: RZ/V2H RPTUN restart task failed: %d\n",
                         ret);
                }
            }

          return OK;
        }
    }

  ret = rpmsg_send(ept, data, len);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: RZ/V2H RPMsg echo failed: %d\n", ret);
      return ret;
    }

  return OK;
}

static void rzv2h_echo_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv)
{
  FAR const char *cpuname = rpmsg_get_cpuname(rdev);
  int ret;

  (void)priv;

  if (g_rzv2h_echo_created || cpuname == NULL ||
      strcmp(cpuname, BOARD_RPTUN_REMOTE_CPUNAME) != 0)
    {
      return;
    }

  ret = rpmsg_create_ept(&g_rzv2h_echo_ept, rdev,
                         BOARD_RPTUN_EPT_NAME, BOARD_RPTUN_EPT_ADDR,
                         RPMSG_ADDR_ANY, rzv2h_echo_ept_cb, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to create %s endpoint: %d\n",
             BOARD_RPTUN_EPT_NAME, ret);
      return;
    }

  g_rzv2h_echo_created = true;
  syslog(LOG_INFO, "RZ/V2H RPMsg endpoint %s created at address %u\n",
         BOARD_RPTUN_EPT_NAME, BOARD_RPTUN_EPT_ADDR);
}

static void rzv2h_echo_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv)
{
  (void)priv;

  if (g_rzv2h_echo_created && g_rzv2h_echo_ept.rdev == rdev)
    {
      rpmsg_destroy_ept(&g_rzv2h_echo_ept);
      g_rzv2h_echo_created = false;
    }
}

#endif /* CONFIG_RZV2H_RPTUN_ECHO */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_rptun_initialize
 ****************************************************************************/

int board_rptun_initialize(void)
{
  int ret;

  if (g_rzv2h_rptun_initialized)
    {
      return -EALREADY;
    }

  g_rzv2h_rptun_config.resource =
    rzv2h_rptun_resource_initialize();

  syslog(LOG_INFO,
         "RZ/V2H RPTUN: channel=%u rsc=%p "
         "vrings=%08lx/%08lx buffer=%08lx+%08lx\n",
         BOARD_RPTUN_CHANNEL,
         g_rzv2h_rptun_config.resource,
         (unsigned long)BOARD_RPTUN_VRING0_ADDR,
         (unsigned long)BOARD_RPTUN_VRING1_ADDR,
         (unsigned long)BOARD_RPTUN_BUFFER_ADDR,
         (unsigned long)BOARD_RPTUN_BUFFER_SIZE);
  syslog(LOG_INFO,
         "RZ/V2H RPTUN: MHU channel=%lu IRQ=%d shmem=%08lx slot=%08lx\n",
         (unsigned long)g_rzv2h_rptun_mhu_cfg.channel,
         g_rzv2h_rptun_mhu_cfg.rx_irq,
         (unsigned long)BOARD_RPTUN_MHU_SHMEM_ADDR,
         (unsigned long)(BOARD_RPTUN_MHU_SHMEM_ADDR +
                         BOARD_RPTUN_MHU_SHMEM_SLOT_SIZE *
                         RZV2H_RPTUN_MHU_CHANNEL));

#ifdef CONFIG_RZV2H_RPTUN_ECHO
  ret = rpmsg_register_callback(NULL, rzv2h_echo_device_created,
                                rzv2h_echo_device_destroy, NULL, NULL);
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = rzv2h_rptun_initialize(&g_rzv2h_rptun_config);
  if (ret < 0)
    {
#ifdef CONFIG_RZV2H_RPTUN_ECHO
      rpmsg_unregister_callback(NULL, rzv2h_echo_device_created,
                                rzv2h_echo_device_destroy, NULL, NULL);
#endif
      return ret;
    }

  g_rzv2h_rptun_initialized = true;
  return OK;
}
