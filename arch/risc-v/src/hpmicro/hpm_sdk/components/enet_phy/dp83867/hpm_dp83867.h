/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_DP83867_DRV_H
#define HPM_DP83867_DRV_H

/*---------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------
 */
#include "hpm_enet_phy.h"
#include "hpm_common.h"
#include "hpm_enet_regs.h"
/*---------------------------------------------------------------------
 *  Macro Const Definitions
 *---------------------------------------------------------------------
 */
#define PHY_ADDR (0U)
#define PHY_ID1  (0x2000U)
#define PHY_ID2  (0x28U)

/*---------------------------------------------------------------------
 *  Typedef Struct Declarations
 *---------------------------------------------------------------------
 */
typedef struct {
    bool loopback;
    uint8_t speed;
    bool auto_negotiation;
    uint8_t duplex;
} dp83867_config_t;

typedef enum {
    DP83867_RX_DELAY_0P25_NS = 0,
    DP83867_RX_DELAY_0P50_NS,
    DP83867_RX_DELAY_0P75_NS,
    DP83867_RX_DELAY_1P00_NS,
    DP83867_RX_DELAY_1P25_NS,
    DP83867_RX_DELAY_1P50_NS,
    DP83867_RX_DELAY_1P75_NS,
    DP83867_RX_DELAY_2P00_NS,
    DP83867_RX_DELAY_2P25_NS,
    DP83867_RX_DELAY_2P50_NS,
    DP83867_RX_DELAY_2P75_NS,
    DP83867_RX_DELAY_3P00_NS,
    DP83867_RX_DELAY_3P25_NS,
    DP83867_RX_DELAY_3P50_NS,
    DP83867_RX_DELAY_3P75_NS,
    DP83867_RX_DELAY_4P00_NS
} dp83867_rgmii_rx_delay_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
/*---------------------------------------------------------------------
 * Exported Functions
 *---------------------------------------------------------------------
 */
void dp83867_reset(ENET_Type *ptr);
void dp83867_basic_mode_default_config(ENET_Type *ptr, dp83867_config_t *config);
bool dp83867_basic_mode_init(ENET_Type *ptr, dp83867_config_t *config);
void dp83867_get_phy_status(ENET_Type *ptr, enet_phy_status_t *status);
void dp83867_set_mdi_crossover_mode(ENET_Type *ptr, enet_phy_crossover_mode_t mode);

#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* HPM_DP83867_H */
