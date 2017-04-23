#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H

#include <stdint.h>
#include <semaphore.h>

void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset);

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H */