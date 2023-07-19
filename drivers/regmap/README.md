drivers/regmap README
========================

This is the README.txt file for the drivers/regmap/ directory.

Contents
========

  - Regmap Header files
	1.     include/nuttx/regmap/regmap.h
	2.     struct lcd_dev_s
	3.     regmap_init
	4.     regmap_init_spi
	5.     regmap_init_i2c
	6.     regmap_exit
	7.     regmap_write
	8.     regmap_bulk_write
	9.     regmap_read
	10.    regmap_bulk_read


Regmap Header files
================

  **include/nuttx/regmap/regmap.h**

      The structures and APIS used in regimap are in this header file.

**struct regmap_bus_s;**

      Each bus must implement an instance of struct regmap_bus_s. That structure defines a call table with the following methods:

    - Single byte reading of the register (8bits)
    typedef CODE int (*reg_read_t)(FAR struct regmap_bus_s *bus,
                                   unsigned int reg,
                                   FAR void *val);

    - Single byte writing of the register (8bits)
    typedef CODE int (*reg_write_t)(FAR struct regmap_bus_s *bus,
                                    unsigned int reg,
                                    unsigned int val);

    - Bulk register data reading.
    typedef CODE int (*read_t)(FAR struct regmap_bus_s *bus,
                               FAR const void *reg_buf, unsigned int reg_size,
                               FAR void *val_buf, unsigned int val_size);

    - Bulk register data writing.
    typedef CODE int (*write_t)(FAR struct regmap_bus_s *bus,
                               FAR const void *data,
                               unsigned int count);

    - Initialize the internal configuration of regmap. The first parameter must be the handle of the bus, and the second parameter is the configuration parameter of the bus. Finally, these two parameters will be transparent to the corresponding bus. If you want to implement the bus interface by yourself, you need to realize the corresponding bus initialization function, refer to regimap_i2c.c and regmap_spi.c.
    FAR struct regmap_s *regmap_init(FAR struct regmap_bus_s *bus,
                                     FAR const struct regmap_config_s *config);

    - Regmap init i2c bus.
    FAR struct regmap_s *regmap_init_i2c(FAR struct i2c_master_s *i2c,
                                         FAR struct i2c_config_s *i2c_config,
                                         FAR const struct regmap_config_s *config);

    - regmap init spi bus.
    FAR struct regmap_s *regmap_init_spi(FAR struct spi_dev_s *spi, uint32_t freq,
                                         uint32_t devid, enum spi_mode_e mode,
                                         FAR const struct regmap_config_s *config);

    - Exit and destroy regmap
    void regmap_exit(FAR struct regmap_s *map);

    - Regmap write() bulk_write() read() bulk_read(), called after initializing the regmap bus device. the first parameter is regmap_s pointer.
    int regmap_write(FAR struct regmap_s *map, unsigned int reg,
                     unsigned int val);
    int regmap_bulk_write(FAR struct regmap_s *map, unsigned int reg,
                          FAR const void *val, unsigned int val_count);
    int regmap_read(FAR struct regmap_s *map, unsigned int reg,
                    FAR void *val);
    int regmap_bulk_read(FAR struct regmap_s *map, unsigned int reg,
                         FAR void *val, unsigned int val_count);

Examples: 
=======================
   BMI160 sensor as an example:
   - Head file
~~~
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>
#include <nuttx/regmap/regmap.h>

#include <stdlib.h>
~~~
   - Define the regmap_s handle in the driver's life cycle
~~~
struct bmi160_dev_s
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  FAR struct regmap_s * regmap;     /* Regmap interface */
#else /* CONFIG_SENSORS_BMI160_SPI */
  FAR struct spi_dev_s *spi;       /* SPI interface */
#endif
};
~~~
   - Initialize regmap
~~~
int bmi160_i2c_regmap_init(FAR struct bmi160_dev_s *priv,
                           FAR struct i2c_master_s *i2c)
{
  struct regmap_config_s config;
  struct i2c_config_s dev_config;

  config.reg_bits = 8;
  config.val_bits = 8;
  config.disable_locking = true;

  dev_config.frequency = BMI160_I2C_FREQ;
  dev_config.address   = BMI160_I2C_ADDR;
  dev_config.addrlen   = 7;

  priv->regmap = regmap_init_i2c(i2c, &dev_config, &config);
  if (priv->regmap == NULL)
    {
      snerr("bmi160 Initialize regmap configuration failed!");
      return -ENXIO;
    }

  return OK;
}
~~~
   -  Use
~~~
  int ret;
  

  ret = regmap_read(priv->regmap, regaddr, &regval);
  if (ret < 0)
    {
      snerr("regmap read address[%2X] failed: %d!\n", regaddr, ret);
    }
   

  ret = regmap_write(priv->regmap, regaddr, regval);
  if (ret < 0)
    {
      snerr("regmap write address[%2X] failed: %d!\n", regaddr, ret);
    }

  ret = regmap_bulk_read(priv->regmap, regaddr, regval, len);
  if (ret < 0)
    {
      snerr("regmap read bulk address[%2X] failed: %d!\n", regaddr, ret);
    }
~~~
