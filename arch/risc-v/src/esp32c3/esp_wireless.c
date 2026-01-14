/****************************************************************************
 * arch/risc-v/src/esp32c3/esp_wireless.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>

#include <debug.h>
#include <assert.h>
#include <netinet/in.h>
#include <sys/param.h>

#include "soc/system_reg.h"
#include "espressif/esp_irq.h"
#include "riscv_internal.h"
#include "esp_private/phy.h"
#include "espressif/esp_hr_timer.h"

#include "periph_ctrl.h"
#include "esp_phy_init.h"
#include "phy_init_data.h"

#include "esp_wireless.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Software Interrupt */

#define SWI_IRQ       ESP_IRQ_FROM_CPU_INTR0
#define SWI_PERIPH    FROM_CPU_INTR0_SOURCE

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Wireless Private Data */

struct esp_wireless_priv_s
{
  volatile int ref;               /* Reference count */

  int cpuint;                     /* CPU interrupt assigned to SWI */

  struct list_node sc_list;       /* Semaphore cache list */
  struct list_node qc_list;       /* Queue cache list */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_swi_irq(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Private data of the wireless common interface */

static struct esp_wireless_priv_s g_esp_wireless_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_swi_irq
 *
 * Description:
 *   Wireless software interrupt callback function.
 *
 * Parameters:
 *   cpuint  - CPU interrupt index
 *   context - Context data from the ISR
 *   arg     - NULL
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int esp_swi_irq(int irq, void *context, void *arg)
{
  int i;
  int ret;
  struct esp_semcache_s *sc;
  struct esp_semcache_s *sc_tmp;
  struct esp_queuecache_s *qc;
  struct esp_queuecache_s *qc_tmp;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  putreg32(0, SYSTEM_CPU_INTR_FROM_CPU_0_REG);

  list_for_every_entry_safe(&priv->sc_list, sc, sc_tmp,
                            struct esp_semcache_s, node)
    {
      for (i = 0; i < sc->count; i++)
        {
          ret = nxsem_post(sc->sem);
          if (ret < 0)
            {
              wlerr("ERROR: Failed to post sem ret=%d\n", ret);
            }
        }

      sc->count = 0;
      list_delete(&sc->node);
    }

  list_for_every_entry_safe(&priv->qc_list, qc, qc_tmp,
                            struct esp_queuecache_s, node)
    {
      ret = file_mq_send(qc->mq_ptr, (const char *)qc->buffer, qc->size, 0);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to send queue ret=%d\n", ret);
        }

      list_delete(&qc->node);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_init_semcache
 *
 * Description:
 *   Initialize semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *   sem - Semaphore data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_semcache(struct esp_semcache_s *sc, sem_t *sem)
{
  sc->sem   = sem;
  sc->count = 0;
  list_initialize(&sc->node);
}

/****************************************************************************
 * Name: esp_post_semcache
 *
 * Description:
 *   Store posting semaphore action into semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_post_semcache(struct esp_semcache_s *sc)
{
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  if (!sc->count)
    {
      list_add_tail(&priv->sc_list, &sc->node);
    }

  sc->count++;

  /* Enable CPU interrupt. This will generate an IRQ as soon as non-IRAM
   * are (re)enabled.
   */

  putreg32(SYSTEM_CPU_INTR_FROM_CPU_0_M, SYSTEM_CPU_INTR_FROM_CPU_0_REG);
}

/****************************************************************************
 * Name: esp_init_queuecache
 *
 * Description:
 *   Initialize queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   mq_ptr - Queue data pointer
 *   buffer - Queue cache buffer pointer
 *   size   - Queue cache buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_queuecache(struct esp_queuecache_s *qc,
                         struct file *mq_ptr,
                         uint8_t *buffer,
                         size_t size)
{
  qc->mq_ptr = mq_ptr;
  qc->size   = size;
  qc->buffer = buffer;
  list_initialize(&qc->node);
}

/****************************************************************************
 * Name: esp_send_queuecache
 *
 * Description:
 *   Store posting queue action and data into queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   buffer - Data buffer
 *   size   - Buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_send_queuecache(struct esp_queuecache_s *qc,
                                   uint8_t *buffer,
                                   int size)
{
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  DEBUGASSERT(qc->size == size);

  list_add_tail(&priv->qc_list, &qc->node);
  memcpy(qc->buffer, buffer, size);

  /* Enable CPU 0 interrupt. This will generate an IRQ as soon as non-IRAM
   * are (re)enabled.
   */

  putreg32(SYSTEM_CPU_INTR_FROM_CPU_0_M, SYSTEM_CPU_INTR_FROM_CPU_0_REG);
}

/****************************************************************************
 * Name: esp_wireless_init
 *
 * Description:
 *   Initialize ESP32-C3 wireless common components for both BT and Wi-Fi.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_init(void)
{
  int ret;
  irqstate_t flags;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  flags = enter_critical_section();
  if (priv->ref != 0)
    {
      priv->ref++;
      leave_critical_section(flags);
      return OK;
    }

  priv->cpuint = esp_setup_irq(SWI_PERIPH, ESP_IRQ_PRIORITY_DEFAULT, 0);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      wlerr("ERROR: Failed to attach IRQ ret=%d\n", ret);
      ret = priv->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  ret = irq_attach(SWI_IRQ, esp_swi_irq, NULL);
  if (ret < 0)
    {
      esp_teardown_irq(SWI_PERIPH, priv->cpuint);
      leave_critical_section(flags);
      wlerr("ERROR: Failed to attach IRQ ret=%d\n", ret);

      return ret;
    }

  list_initialize(&priv->sc_list);
  list_initialize(&priv->qc_list);

  up_enable_irq(SWI_IRQ);

  priv->ref++;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_wireless_deinit
 *
 * Description:
 *   De-initialize ESP32-C3 wireless common components.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_deinit(void)
{
  irqstate_t flags;
  struct esp_wireless_priv_s *priv = &g_esp_wireless_priv;

  flags = enter_critical_section();

  if (priv->ref > 0)
    {
      priv->ref--;
      if (priv->ref == 0)
        {
          up_disable_irq(SWI_IRQ);
          irq_detach(SWI_IRQ);
          esp_teardown_irq(SWI_PERIPH, priv->cpuint);
        }
    }

  leave_critical_section(flags);

  return OK;
}
