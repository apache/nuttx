/****************************************************************************
 * include/nuttx/modem/altmdm.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_MODEM_ALTMDM_H
#define __INCLUDE_NUTTX_MODEM_ALTMDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <queue.h>
#include <debug.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
#  define m_err     _err
#  define m_info    _info
#else
#  define m_err(x...)
#  define m_info(x...)
#endif

#define MODEM_IOC_POWERON             _MODEMIOC(1)
#define MODEM_IOC_POWEROFF            _MODEMIOC(2)
#define MODEM_IOC_READABORT           _MODEMIOC(3)
#define MODEM_IOC_SLEEP               _MODEMIOC(4)
#define MODEM_IOC_PM_REGISTERCB       _MODEMIOC(5)
#define MODEM_IOC_PM_DEREGISTERCB     _MODEMIOC(6)
#define MODEM_IOC_PM_GETSTATE         _MODEMIOC(7)
#define MODEM_IOC_PM_INITWAKELOCK     _MODEMIOC(8)
#define MODEM_IOC_PM_ACQUIREWAKELOCK  _MODEMIOC(9)
#define MODEM_IOC_PM_RELEASEWAKELOCK  _MODEMIOC(10)
#define MODEM_IOC_PM_GETNUMOFWAKELOCK _MODEMIOC(11)
#define MODEM_IOC_PM_GETWAKELOCKSTATE _MODEMIOC(12)
#define MODEM_IOC_PM_ERR_REGISTERCB   _MODEMIOC(13)
#define MODEM_IOC_PM_ERR_DEREGISTERCB _MODEMIOC(14)

#define MODEM_PM_CB_TYPE_NORMAL       0
#define MODEM_PM_CB_TYPE_ERROR        1
#define MODEM_PM_STATE_SLEEP          0
#define MODEM_PM_STATE_WAKE           1
#define MODEM_PM_ERR_RESET_BOOTSTAT_NONE      0x00
#define MODEM_PM_ERR_RESET_BOOTSTAT_BOOTING   0x01
#define MODEM_PM_ERR_RESET_BOOTSTAT_UPDATING  0x02
#define MODEM_PM_ERR_RESET_BOOTSTAT_DONE      0x10

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altmdm_pm_wakelock_s
  {
    sq_entry_t queue;
    int count;
  };

struct altmdm_lower_s
{
  void (*poweron)(void);
  void (*poweroff)(void);
  void (*sready_irqattach)(bool attach, xcpt_t handler);
  void (*sready_irqenable)(bool enable);
  bool (*sready)(void);
  void (*master_request)(bool request);
  void (*wakeup)(bool wakeup);
  uint32_t (*spi_maxfreq)(void);
};

typedef void (*altmdm_pm_cbfunc_t) (uint32_t state);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: altmdm_register
 *
 * Description:
 *   Register the ALTMDM character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/altmdm".
 *   dev     - An instance of the SPI interface to use to communicate with
 *             ALTMDM.
 *   lower   - An instance of the lower interface.
 *
 * Returned Value:
 *   Not NULL on success; NULL on failure.
 *
 ****************************************************************************/

FAR void *altmdm_register(FAR const char *devpath, FAR struct spi_dev_s *dev,
                          FAR const struct altmdm_lower_s *lower);

/****************************************************************************
 * Name: altmdm_unregister
 *
 * Description:
 *   Unregister the ALTMDM character device.
 *
 * Input Parameters:
 *   handle - The pointer that getting from altmdm_register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altmdm_unregister(FAR void *handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MODEM_ALTMDM_H */
