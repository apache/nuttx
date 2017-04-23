#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H

#include "bcmf_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include <queue.h>
#include <semaphore.h>
#include <nuttx/sdio.h>

#include "bcmf_sdio_core.h"

/* sdio chip configuration structure */

struct bcmf_sdio_chip {
	uint32_t ram_size;
	uint32_t core_base[MAX_CORE_ID];

	uint8_t      *firmware_image;
	unsigned int *firmware_image_size;

	uint8_t      *nvram_image;
	unsigned int *nvram_image_size;
};

/* sdio bus structure extension */

struct bcmf_sdio_dev_s {
  struct bcmf_bus_dev_s bus;       /* Default bcmf bus structure */
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */

  struct bcmf_sdio_chip *chip;     /* Chip specific configuration */

  volatile bool ready;             /* Current device status */
  bool sleeping;                   /* Current sleep status */

  int thread_id;                   /* Processing thread id */
  sem_t thread_signal;             /* Semaphore for processing thread event */
  struct wdog_s *waitdog;          /* Processing thread waitdog */

  uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

  volatile bool irq_pending;       /* True if interrupt is pending */
  uint32_t intstatus;              /* Copy of device current interrupt status */

  uint8_t max_seq;                 /* Maximum transmit sequence allowed */
  uint8_t tx_seq;                  /* Transmit sequence number (next) */
  uint8_t rx_seq;                  /* Receive sequence number (expected) */

  sem_t tx_queue_mutex;            /* Lock for transmit queue */
  dq_queue_t tx_queue;             /* Queue of frames to tramsmit */
};

int bcmf_bus_sdio_initialize(FAR struct bcmf_dev_s *priv,
					int minor, FAR struct sdio_dev_s *dev);

/* FIXME: Low level bus data transfer function
 * To avoid bus error, len will be aligned to:
 * - upper power of 2 iflen is lesser than 64
 * - upper 64 bytes block if len is greater than 64
 */

int bcmf_transfer_bytes(FAR struct bcmf_sdio_dev_s *sbus, bool write,
                         uint8_t function, uint32_t address,
                         uint8_t *buf, unsigned int len);

int bcmf_read_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                  uint32_t address, uint8_t *reg);

int bcmf_write_reg(FAR struct bcmf_sdio_dev_s *sbus, uint8_t function,
                   uint32_t address, uint8_t reg);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H */