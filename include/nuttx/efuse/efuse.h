/****************************************************************************
 * include/nuttx/efuse/efuse.h
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

#ifndef __INCLUDE_NUTTX_EFUSE_EFUSE_H
#define __INCLUDE_NUTTX_EFUSE_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <signal.h>
#include <stdint.h>

#ifdef CONFIG_EFUSE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     EFUSEIOC_READ_FIELD
 * Description: Read a blob of bits from an efuse field.
 * Arguments:   A structure containing the field[], a dst pointer, and size
 *              of bits to be read from efuses.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_READ_FIELD      _EFUSEIOC(0x0001)

/* Command:     EFUSEIOC_WRITE_FIELD
 * Description: Write a blob of bits to an efuse's field
 * Arguments:   A structure containing the field[], the src memory and the
 *              amount of bits to write.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_WRITE_FIELD     _EFUSEIOC(0x0002)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure eFuse field */

struct efuse_desc_s
{
  uint16_t   bit_offset; /* Bit offset related to beginning of efuse */
  uint16_t   bit_count;  /* Length of bit field */
};

/* Type definition for an eFuse field */

typedef struct efuse_desc_s efuse_desc_t;

/* Structs with the parameters passed to the IOCTLs */

/* The efuse_param is used by the application to inform which field(s)
 * will be passed to the ioctl() operation.
 *   - 'efuse_desc_t *field[]': contains one or more field and it
 *     it terminated by a NULL to indicate there is no more fields.
 *     NOTE: normally the application don't need to create these fields
 *           they are mapped inside an arch efuse table and referenced
 *           in a header file inside include/nuttx/efuse/ directory.
 *   - size: how many bits the field(s) use
 *   - data: an application side allocated buffer where the read operation
 *           will store the efuse bits, so the number of allocated bytes
 *           need to be enough to store all bits. For write operation, this
 *           pointer contains the bits to be written.
 *
 *  FINAL NOTE: The bit order is architecture dependent (tested only on
 *              little endian arch, because I don't have a big endian arch
 *              supported by NuttX. So, the first efuse bit starting at
 *              'bit_offset' will be store at bit 0 of the data[0] and so
 *              on.
 */

struct efuse_param
{
  FAR const efuse_desc_t **field;
  size_t  size;
  FAR uint8_t *data;
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct efuse_lowerhalf_s;
struct efuse_ops_s
{
  /* Required methods *******************************************************/

  /* Read an EFUSE bit */

  CODE int (*read_field)(FAR struct efuse_lowerhalf_s *lower,
                         FAR const efuse_desc_t *field[],
                         FAR uint8_t *data, size_t bit_size);

  /* Write an EFUSE bit */

  CODE int (*write_field)(FAR struct efuse_lowerhalf_s *lower,
                          FAR const efuse_desc_t *field[],
                          FAR const uint8_t *data, size_t bit_size);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct efuse_lowerhalf_s *lower, int cmd,
                    unsigned long arg);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct efuse_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct efuse_ops_s  *ops;  /* Lower half operations */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *efuse_register(FAR const char *path,
                         FAR struct efuse_lowerhalf_s *lower);

void efuse_unregister(FAR void *handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_EFUSE */
#endif /* __INCLUDE_NUTTX_EFUSE_EFUSE_H */
