/****************************************************************************
 * arch/arm/include/cxd56xx/cisif.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_CISIF_H
#define __ARCH_ARM_INCLUDE_CXD56XX_CISIF_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*notify_callback_t)(uint8_t code,
                                  uint32_t size,
                                  uint32_t addr);
typedef void (*comp_callback_t)(uint8_t code,
                                uint32_t size,
                                uint32_t addr);

struct cisif_init_yuv_param_s
{
  uint16_t          hsize;
  uint16_t          vsize;
  uint32_t          notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_init_yuv_param_s cisif_init_yuv_param_t;

struct cisif_init_jpeg_param_s
{
  uint32_t notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_init_jpeg_param_s cisif_init_jpeg_param_t;

struct cisif_sarea_s
{
  uint8_t *strg_addr;
  uint32_t strg_size;
};

typedef struct cisif_sarea_s cisif_sarea_t;

struct cisif_param_s
{
  uint32_t                format;
  cisif_init_yuv_param_t  yuv_param;
  cisif_init_jpeg_param_t jpg_param;
  cisif_sarea_t           sarea;
  comp_callback_t         comp_func;
};

typedef struct cisif_param_s cisif_param_t;

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cxd56_cisifinit(void);
int cxd56_cisiffinalize(void);
int cxd56_cisifstartcapture(cisif_param_t *param, cisif_sarea_t *sarea);
int cxd56_cisifstopcapture(void);
int cxd56_cisifsetdmabuf(cisif_sarea_t *sarea);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_CISIF_H */
