/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_psave.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_PSAVE_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_PSAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int32_t altcom_getedrx_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_setedrx_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_getdedrx_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz,
                                    FAR uint16_t *altcid);
int32_t altcom_getpsm_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_setpsm_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_getdpsm_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_getce_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_setce_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid);
int32_t altcom_getedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_setedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_getdedrx_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_getpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_setpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_getdpsm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                 FAR uint8_t *pktbuf, size_t pktsz,
                                 uint8_t altver, FAR void **arg,
                                 size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_getce_pkt_parse(FAR struct alt1250_dev_s *dev,
                               FAR uint8_t *pktbuf, size_t pktsz,
                               uint8_t altver, FAR void **arg,
                               size_t arglen, FAR uint64_t *bitmap);
int32_t altcom_setce_pkt_parse(FAR struct alt1250_dev_s *dev,
                               FAR uint8_t *pktbuf, size_t pktsz,
                               uint8_t altver, FAR void **arg,
                               size_t arglen, FAR uint64_t *bitmap);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_PSAVE_H */
