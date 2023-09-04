/****************************************************************************
 * drivers/math/math.c
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

#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/math/math.h>
#include <nuttx/math/math_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

typedef struct math_config_s math_upperhalf_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t math_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t math_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     math_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_math_ops =
{
  NULL,         /* open */
  NULL,         /* close */
  math_read,    /* read */
  math_write,   /* write */
  NULL,         /* seek */
  math_ioctl,   /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: math_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t math_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: math_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t math_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: math_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the math timer
 *   work is done.
 *
 ****************************************************************************/

static int math_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode              *inode =  filep->f_inode;
  FAR math_upperhalf_s          *upper =  inode->i_private;
  int                            ret   = -ENOTTY;
  irqstate_t                     flags;

  DEBUGASSERT(upper != NULL);

  _info("cmd: %d arg: %lu\n", cmd, arg);

  flags = enter_critical_section();

  switch (cmd)
    {
      /* CORDIC calulcate */

#ifdef CONFIG_MATH_CORDIC
      case MATHIOC_CORDIC_CALC:
        {
          FAR struct cordic_calc_s *calc =
            (FAR struct cordic_calc_s *)((uintptr_t)arg);

          if (upper->cordic != NULL)
            {
              ret = upper->cordic->ops->calc(upper->cordic, calc);
            }

          break;
        }
#endif

#ifdef CONFIG_MATH_FFT
      case MATHIOC_FFT_CALC:
        {
          FAR struct fft_calc_s *calc =
            (FAR struct fft_calc_s *)((uintptr_t)arg);

          if (upper->fft != NULL)
            {
              ret = upper->fft->ops->calc(upper->fft, calc);
            }

          break;
        }
#endif

#ifdef CONFIG_MATH_MPI
      case MATHIOC_MPI_CALC:
        {
          FAR struct mpi_calc_s *calc =
            (FAR struct mpi_calc_s *)((uintptr_t)arg);

          if (upper->mpi != NULL)
            {
              ret = upper->mpi->ops->calc(upper->mpi, calc);
            }

          break;
        }
#endif
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: math_register
 *
 * Description:
 *   Register a math driver.
 *
 ****************************************************************************/

int math_register(FAR const char *path,
                  FAR const struct math_config_s *config)
{
  FAR math_upperhalf_s *upper =  NULL;
  int                   ret   = -ENOMEM;

  DEBUGASSERT(path);
  DEBUGASSERT(config);

  /* Allocate the upper-half data structure */

  upper = (FAR math_upperhalf_s *)
    kmm_malloc(sizeof(math_upperhalf_s));
  if (!upper)
    {
      _err("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the math device structure */

  memcpy(upper, config, sizeof(math_upperhalf_s));

  /* Register the math timer device */

  ret = register_driver(path, &g_math_ops, 0666, upper);
  if (ret < 0)
    {
      _err("register math driver failed: %d\n", ret);
      kmm_free(upper);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: cordic_register
 *
 * Description:
 *   Register a CORDIC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_MATH_CORDIC
int cordic_register(FAR const char *path,
                    FAR struct cordic_lowerhalf_s *lower)
{
  struct math_config_s config;

  memset(&config, 0, sizeof(config));
  config.cordic = lower;

  return math_register(path, &config);
}
#endif

/****************************************************************************
 * Name: fft_register
 *
 * Description:
 *   Register a FFT driver.
 *
 ****************************************************************************/

#ifdef CONFIG_MATH_FFT
int fft_register(FAR const char *path,
                    FAR struct fft_lowerhalf_s *lower)
{
  struct math_config_s config;

  memset(&config, 0, sizeof(config));
  config.fft = lower;

  return math_register(path, &config);
}
#endif

/****************************************************************************
 * Name: mpi_register
 *
 * Description:
 *   Register a MPI driver.
 *
 ****************************************************************************/

#ifdef CONFIG_MATH_MPI
int mpi_register(FAR const char *path,
                 FAR struct mpi_lowerhalf_s *lower)
{
  struct math_config_s config;

  memset(&config, 0, sizeof(config));
  config.mpi = lower;

  return math_register(path, &config);
}
#endif
