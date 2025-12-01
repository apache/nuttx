/****************************************************************************
 * boards/arm64/fvp-v8r/fvp-armv8r/src/fvp_boardinit.c
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
#include <stdint.h>
#include <nuttx/board.h>

#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun_bmp.h>

#include "fvp-armv8r.h"
#include "arm64_gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RPTUN_BMP
#define QEMU_RSC_TABLE_SIZE 0x10000
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RPTUN_BMP
struct rptun_rsc
{
  struct rptun_rsc_s rsc;
  uint8_t padding[QEMU_RSC_TABLE_SIZE - sizeof(struct rptun_rsc_s)];
};

static struct rptun_rsc g_rptun_rsc[3];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fvp_memory_initialize
 *
 * Description:
 *   All vdk architectures must provide the following entry point.  This
 *   entry point is called early in the initialization before memory has
 *   been configured.  This board-specific function is responsible for
 *   configuring any on-board memories.
 *
 *   Logic in vdk_memory_initialize must be careful to avoid using any
 *   global variables because those will be uninitialized at the time this
 *   function is called.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void fvp_memory_initialize(void)
{
  /* SDRAM was initialized by a bootloader in the supported configurations. */
}

/****************************************************************************
 * Name: vdk_board_initialize
 *
 * Description:
 *   All vdk architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   vdk_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void fvp_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

#endif
}

#ifdef CONFIG_RPTUN_BMP
static void rptun_setup_shmem(struct rptun_rsc_s *rsc,
                              const char *host_cpuname,
                              const char *remote_cpuname)
{
  memset(rsc, 0, sizeof(struct rptun_rsc_s));
  rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                           rpmsg_vdev);
  rsc->rpmsg_vdev.type          = RSC_VDEV;
  rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
  rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                | 1 << VIRTIO_RPMSG_F_ACK
                                | 1 << VIRTIO_RPMSG_F_BUFSZ
                                | 1 << VIRTIO_RPMSG_F_CPUNAME;
  rsc->rpmsg_vdev.num_of_vrings = 2;
  rsc->rpmsg_vdev.notifyid      = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
  rsc->rpmsg_vdev.reserved[0]   = VIRTIO_DEV_DRIVER;
  rsc->rpmsg_vdev.reserved[1]   = 0;
  rsc->rpmsg_vring0.align       = 8;
  rsc->rpmsg_vring0.num         = 8;
  rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vring0.da          = 0;
  rsc->rpmsg_vring1.align       = 8;
  rsc->rpmsg_vring1.num         = 8;
  rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vring1.da          = 0;
  rsc->config.r2h_buf_size      = 0x200;
  rsc->config.h2r_buf_size      = 0x200;
  strlcpy((char *)rsc->config.host_cpuname, host_cpuname,
          VIRTIO_RPMSG_CPUNAME_SIZE);
  strlcpy((char *)rsc->config.remote_cpuname, remote_cpuname,
          VIRTIO_RPMSG_CPUNAME_SIZE);

  rsc->offset[1]                = offsetof(struct rptun_rsc_s,
                                           carveout);
  rsc->carveout.type            = RSC_CARVEOUT;
  rsc->carveout.da              = (metal_phys_addr_t)rsc +
                                  ALIGN_UP(sizeof(struct rptun_rsc_s), 8);
  rsc->carveout.pa              = FW_RSC_U32_ADDR_ANY;
  rsc->carveout.len             = 0x200 * 8 * 2 + 0x1000;
  memcpy(rsc->carveout.name, "vdev0buffer", 11);

  rsc->rsc_tbl_hdr.num          = 2;
  UP_DMB();
  rsc->rsc_tbl_hdr.ver          = 1;
}

static int fvp_rpmsg_virtio_init(void)
{
  struct rptun_rsc_s *rsc = &g_rptun_rsc[0].rsc;
  cpu_set_t cpuset;
  int ret = 0;

  CPU_ZERO(&cpuset);

  switch (this_cpu())
    {
      case 0:
        rptun_setup_shmem(rsc, "core0", "core1");
        CPU_SET(1, &cpuset);
        ret = rptun_bmp_init("core1", true, rsc, GIC_IRQ_SGI15,
                            GIC_IRQ_SGI14, cpuset);
        DEBUGASSERT(ret >= 0);

        rsc = &g_rptun_rsc[1].rsc;
        rptun_setup_shmem(rsc, "core0", "core2");
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);
        ret = rptun_bmp_init("core2", true, rsc, GIC_IRQ_SGI13,
                            GIC_IRQ_SGI4, cpuset);
        DEBUGASSERT(ret >= 0);

        rsc = &g_rptun_rsc[2].rsc;
        rptun_setup_shmem(rsc, "core0", "core3");
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        ret = rptun_bmp_init("core3", true, rsc, GIC_IRQ_SGI5,
                            GIC_IRQ_SGI6, cpuset);
        DEBUGASSERT(ret >= 0);
        break;
      case 1:
        rsc = &g_rptun_rsc[0].rsc;
        CPU_SET(0, &cpuset);
        ret = rptun_bmp_init("core0", false, rsc, GIC_IRQ_SGI14,
                            GIC_IRQ_SGI15, cpuset);
        DEBUGASSERT(ret >= 0);
        break;
      case 2:
        rsc = &g_rptun_rsc[1].rsc;
        CPU_SET(0, &cpuset);
        ret = rptun_bmp_init("core0", false, rsc, GIC_IRQ_SGI4,
                            GIC_IRQ_SGI13, cpuset);
        DEBUGASSERT(ret >= 0);
        break;
      case 3:
        rsc = &g_rptun_rsc[2].rsc;
        CPU_SET(0, &cpuset);
        ret = rptun_bmp_init("core0", false, rsc, GIC_IRQ_SGI6,
                            GIC_IRQ_SGI5, cpuset);
        DEBUGASSERT(ret >= 0);
        break;
      default:
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  fvp_bringup();
#ifdef CONFIG_RPTUN_BMP
  fvp_rpmsg_virtio_init();
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
