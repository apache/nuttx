/****************************************************************************
 * drivers/regmap/regmap.c
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

#include <nuttx/regmap/regmap.h>
#include <nuttx/lib/math32.h>
#include <nuttx/kmalloc.h>

#include <debug.h>

#include "internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REGMAP_ALIGNED(n, a) (((uint32_t)(n) & ((a) - 1)) == 0)
#define REGMAP_DIVUP(n,d) (((n) + (d) - 1) / (d))

#define REGMAP_DEFAULT_BIT 8

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void regmap_lock_unlock_none(FAR void *context)
{
}

static void regmap_lock_mutex(FAR void *context)
{
  FAR struct regmap_s *map = context;
  nxmutex_lock(&map->mutex[0]);
}

static void regmap_unlock_mutex(FAR void *context)
{
  FAR struct regmap_s *map = context;
  nxmutex_unlock(&map->mutex[0]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regmap_init
 *
 * Description:
 *   Initialize the internal configuration of regmap. The first parameter
 *   must be the handle of the bus, and the second parameter is the
 *   configuration parameter of the bus. Finally, these two parameters will
 *   be transparent to the corresponding bus.
 *
 * Input Parameters:
 *   dev    - device handle.
 *   bus    - device configuration.
 *   config - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

FAR struct regmap_s *regmap_init(FAR struct regmap_bus_s *bus,
                                 FAR const struct regmap_config_s *config)
{
  FAR struct regmap_s *map;

  if (config == NULL || bus == NULL)
    {
      return NULL;
    }

  if (config->disable_locking)
    {
      map = kmm_zalloc(sizeof(*map));
      if (map == NULL)
        {
          return NULL;
        }

      map->lock   = regmap_lock_unlock_none;
      map->unlock = regmap_lock_unlock_none;
    }
  else
    {
      map = kmm_zalloc(sizeof(*map) + sizeof(mutex_t));
      if (map == NULL)
        {
          return NULL;
        }

      nxmutex_init(&map->mutex[0]);
      map->lock   = regmap_lock_mutex;
      map->unlock = regmap_unlock_mutex;
    }

  if (config->reg_stride != 0)
    {
      map->reg_stride = config->reg_stride;
    }
  else
    {
      map->reg_stride = 1;
    }

  map->reg_bytes = REGMAP_DIVUP(config->reg_bits, REGMAP_DEFAULT_BIT);
  map->val_bytes = REGMAP_DIVUP(config->val_bits, REGMAP_DEFAULT_BIT);

  map->bus = bus;
  map->reg_read  = bus->reg_read;
  map->reg_write = bus->reg_write;
  map->read  = bus->read;
  map->write = bus->write;

  return map;
}

/****************************************************************************
 * Name: regmap_write
 *
 * Description:
 *   Regmap write, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map - regmap handler, from regmap bus init function return.
 *   reg - register address to be write.
 *   val - write data.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_write(FAR struct regmap_s *map, unsigned int reg,
                 unsigned int val)
{
  int ret;

  DEBUGASSERT(REGMAP_ALIGNED(reg, map->reg_stride));

  map->lock(map);

  ret = map->reg_write(map->bus, reg, val);

  map->unlock(map);

  return ret;
}

/****************************************************************************
 * Name: regmap_bulk_write
 *
 * Description:
 *   Regmap bulk write, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map       - regmap handler, from regmap bus init function return.
 *   reg       - register address to be write.
 *   val       - write data buffer.
 *   val_count - write data buffer size.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_bulk_write(FAR struct regmap_s *map, unsigned int reg,
                      FAR const void *val, unsigned int val_count)
{
  size_t val_bytes = map->val_bytes;
  int ret = -ENOSYS;
  unsigned int ival;
  FAR uint8_t *ptr;
  int i;

  DEBUGASSERT(REGMAP_ALIGNED(reg, map->reg_stride));

  map->lock(map);
  if (map->write != NULL)
    {
      ret = map->write(map->bus, val, val_bytes * val_count);
      goto out;
    }

  for (i = 0; i < val_count; i++)
    {
      ptr = (FAR uint8_t *)val + (i * val_bytes);
      switch (val_bytes)
        {
          case 1:
            ival = *(FAR uint8_t *)ptr;
            break;
          case 2:
            ival = *(FAR uint16_t *)ptr;
            break;
          case 4:
            ival = *(FAR uint32_t *)ptr;
            break;
          default:
            ret = -EINVAL;
            goto out;
        }

      ret = map->reg_write(map->bus, reg + (i * map->reg_stride), ival);
      if (ret < 0)
        {
          break;
        }
    }

out:
  map->unlock(map);
  return ret;
}

/****************************************************************************
 * Name: regmap_read
 *
 * Description:
 *   Regmap read, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map - regmap handler, from regmap bus init function return.
 *   reg - register address to be read.
 *   val - read data buffer.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_read(FAR struct regmap_s *map, unsigned int reg, FAR void *val)
{
  int ret;

  DEBUGASSERT(REGMAP_ALIGNED(reg, map->reg_stride));

  map->lock(map);

  ret = map->reg_read(map->bus, reg, val);

  map->unlock(map);
  return ret;
}

/****************************************************************************
 * Name: regmap_bulk_read
 *
 * Description:
 *   Regmap bulk read, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map       - regmap handler, from regmap bus init function return.
 *   reg       - register address to be read.
 *   val       - read data buffer.
 *   val_count - read data buffer size.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_bulk_read(FAR struct regmap_s *map, unsigned int reg,
                     FAR void *val, unsigned int val_count)
{
  FAR uint32_t *u32 = val;
  FAR uint16_t *u16 = val;
  FAR uint8_t  *u8  = val;
  unsigned int ival;
  int ret = -ENOSYS;
  int i;

  DEBUGASSERT(REGMAP_ALIGNED(reg, map->reg_stride));

  map->lock(map);

  if (map->read != NULL)
    {
      ret = map->read(map->bus, &reg, map->reg_bytes, val, val_count);
    }
  else
    {
      for (i = 0; i < val_count; i++)
        {
          ret = map->reg_read(map->bus, reg + (i * map->reg_stride), &ival);
          if (ret < 0)
            {
              break;
            }

          switch (map->val_bytes)
            {
              case 4:
                u32[i] = ival;
                break;
              case 2:
                u16[i] = ival;
                break;
              case 1:
                u8[i] = ival;
                break;
              default:
                map->unlock(map);
                return -EINVAL;
            }
        }
    }

  map->unlock(map);

  return ret;
}

/****************************************************************************
 * Name: regmap_exit
 *
 * Description:
 *   Free a previously allocated register map
 *
 * Input Parameters:
 *   map - regmap handler, from regmap bus init function return.
 *
 * Assumptions/Limitations:
 *   None.
 ****************************************************************************/

void regmap_exit(FAR struct regmap_s *map)
{
  if (!map->disable_locking)
    {
      nxmutex_destroy(&map->mutex[0]);
    }

  kmm_free(map->bus);
  kmm_free(map);
}
