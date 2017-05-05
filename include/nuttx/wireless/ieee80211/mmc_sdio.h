#ifndef caca
#define caca
#include <stdint.h>
#include <nuttx/sdio.h>

int sdio_probe(FAR struct sdio_dev_s *dev);

int sdio_set_wide_bus(struct sdio_dev_s *dev);

int sdio_set_blocksize(FAR struct sdio_dev_s *dev, uint8_t function,
                       uint16_t blocksize);

int sdio_enable_function(FAR struct sdio_dev_s *dev, uint8_t function);

int sdio_enable_interrupt(FAR struct sdio_dev_s *dev, uint8_t function);

int sdio_sendcmdpoll(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg);

int sdio_io_rw_direct(FAR struct sdio_dev_s *dev, bool write,
                      uint8_t function, uint32_t address,
                      uint8_t inb, uint8_t* outb);

int sdio_io_rw_extended(FAR struct sdio_dev_s *dev, bool write,
                        uint8_t function, uint32_t address,
                        bool inc_addr, uint8_t *buf,
                        unsigned int blocklen, unsigned int nblocks);

#endif
