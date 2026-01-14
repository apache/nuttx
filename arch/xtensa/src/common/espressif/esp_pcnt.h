/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_pcnt.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_PCNT_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_PCNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_PCNT_CHAN_INVERT_EDGE_IN    (1 << 0)
#define ESP_PCNT_CHAN_INVERT_LVL_IN     (1 << 1)
#define ESP_PCNT_CHAN_VIRT_EDGE_IO_LVL  (1 << 2)
#define ESP_PCNT_CHAN_VIRT_LVL_IO_LVL   (1 << 3)
#define ESP_PCNT_CHAN_IO_LOOPBACK       (1 << 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp_pcnt_chan_level_action_e
{
    ESP_PCNT_CHAN_LEVEL_ACTION_KEEP,    /* Keep current count mode */
    ESP_PCNT_CHAN_LEVEL_ACTION_INVERSE, /* Invert current count mode */
    ESP_PCNT_CHAN_LEVEL_ACTION_HOLD,    /* Hold current count value */
};

enum esp_pcnt_chan_edge_action_e
{
    ESP_PCNT_CHAN_EDGE_ACTION_HOLD,     /* Hold current count value */
    ESP_PCNT_CHAN_EDGE_ACTION_INCREASE, /* Increase count value */
    ESP_PCNT_CHAN_EDGE_ACTION_DECREASE, /* Decrease count value */
};

enum esp_pcnt_unit_zero_cross_mode_e
{
    ESP_PCNT_UNIT_ZERO_CROSS_POS_ZERO, /* From positive value to zero */
    ESP_PCNT_UNIT_ZERO_CROSS_NEG_ZERO, /* From negative value, to zero */
    ESP_PCNT_UNIT_ZERO_CROSS_NEG_POS,  /* From negative value, to positive value */
    ESP_PCNT_UNIT_ZERO_CROSS_POS_NEG,  /* From positive value, to negative value */
};

struct esp_pcnt_unit_config_s
{
    int low_limit;      /* Low limitation of the count unit */
    int high_limit;     /* High limitation of the count unit */
    bool accum_count;   /* Accumulate the count value when overflow flag */
};

struct esp_pcnt_chan_config_s
{
    int edge_gpio_num;  /* Edge signal GPIO number,-1 if unused */
    int level_gpio_num; /* Level signal GPIO number ,-1 if unused */
    int flags;          /* Channel config flags */
};

struct esp_pcnt_watch_event_data_s
{
    int unit_id;                                            /* PCNT Unit id */
    int watch_point_value;                                  /* Watch point value that triggered the event */
    enum esp_pcnt_unit_zero_cross_mode_e zero_cross_mode;   /* Zero cross mode */
};

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_pcnt_new_unit
 *
 * Description:
 *   Request PCNT unit and config it with given parameters.
 *
 * Input Parameters:
 *   config - PCNT unit configuration
 *
 * Returned Value:
 *   PCNT unit number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *esp_pcnt_new_unit(
    struct esp_pcnt_unit_config_s *config);

/****************************************************************************
 * Name: esp_pcnt_del_unit
 *
 * Description:
 *   Delete PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_del_unit(struct cap_lowerhalf_s *dev);

/****************************************************************************
 * Name: esp_pcnt_unit_add_watch_point
 *
 * Description:
 *   Add watch point to given PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   ret - Value to watch
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_unit_add_watch_point(struct cap_lowerhalf_s *dev,
                                  int watch_point);

/****************************************************************************
 * Name: esp_pcnt_unit_remove_watch_point
 *
 * Description:
 *   Remove watch point from given PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   ret - Watch point value to remove
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_unit_remove_watch_point(struct cap_lowerhalf_s *dev,
                                     int watch_point);

/****************************************************************************
 * Name: esp_pcnt_new_channel
 *
 * Description:
 *   Request channel on given PCNT unit and config it with given parameters.
 *
 * Input Parameters:
 *   dev    - Pointer to the pcnt driver struct
 *   config - PCNT unit channel configuration
 *
 * Returned Value:
 *   PCNT unit channel number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

int esp_pcnt_new_channel(struct cap_lowerhalf_s *dev,
                         struct esp_pcnt_chan_config_s *config);

/****************************************************************************
 * Name: esp_pcnt_del_channel
 *
 * Description:
 *   Delete PCNT unit channel.
 *
 * Input Parameters:
 *   channel - Channel number to delete
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_del_channel(int channel);

/****************************************************************************
 * Name: esp_pcnt_channel_set_edge_action
 *
 * Description:
 *   Set channel actions when edge signal changes.
 *
 * Input Parameters:
 *   channel  - Channel number to set actions
 *   post act - Action on posedge signal
 *   neg_act  - Action on negedge signal
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

void esp_pcnt_channel_set_edge_action(int channel,
    enum esp_pcnt_chan_edge_action_e pos_act,
    enum esp_pcnt_chan_edge_action_e neg_act);

/****************************************************************************
 * Name: esp_pcnt_channel_set_level_action
 *
 * Description:
 *   Set channel actions when level signal changes.
 *
 * Input Parameters:
 *   channel  - Channel number to set actions
 *   post act - Action on posedge signal
 *   neg_act  - Action on negedge signal
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

void esp_pcnt_channel_set_level_action(int channel,
    enum esp_pcnt_chan_level_action_e pos_act,
    enum esp_pcnt_chan_level_action_e neg_act);

#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_PCNT_H */
