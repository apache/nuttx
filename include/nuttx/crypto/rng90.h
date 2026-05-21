#ifndef __INCLUDE_NUTTX_CRYPTO_RNG90_H
#define __INCLUDE_NUTTX_CRYPTO_RNG90_H

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <stdint.h>

#if defined(CONFIG_I2C) && defined(CONFIG_DEV_RNG90)

#define RNG90_I2C_ADDR 0x40
#define RNG90_I2C_FREQ 400000

#define _RNG90IOCBASE   (0x3b00) /* RNG90 ioctl base */
#define _RNG90IOC(nr)   _IOC(_RNG90IOCBASE, nr)

#define RNG90_IOC_WAKEUP   _RNG90IOC(0x01)
#define RNG90_IOC_SLEEP    _RNG90IOC(0x02)
#define RNG90_IOC_GENRND   _RNG90IOC(0x03)
#define RNG90_IOC_SELFTEST _RNG90IOC(0x04)

struct i2c_master_s;

struct rng90_selftest_s
{
  uint8_t mode;
  uint8_t result;
};

int rng90_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c,
                    uint8_t addr);
                    
#endif /* CONFIG_I2C && CONFIG_DEV_RNG90 */
#endif /* __INCLUDE_NUTTX_CRYPTO_RNG90_H */