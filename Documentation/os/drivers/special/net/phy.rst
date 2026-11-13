==================
PHY Device Drivers
==================

These drivers use the :doc:`PHYLIB framework </os/networking/phylib>`.

Realtek RTL8201F and RTL8211F
=============================

``CONFIG_NET_PHY_REALTEK_RTL8201F`` enables the Clause 22 MII/RMII driver;
``CONFIG_NET_PHY_REALTEK_RTL8211F`` enables the RTL8211F-VD and RTL8211F/FDI
RGMII driver. Register them with ``rtl8201f_register()`` or
``rtl8211f_register()`` from ``<nuttx/net/realtek_phy.h>`` before creating
PHY devices. Repeated registration is safe.

Set ``phydev->interface`` before ``phy_connect()``. For RTL8201F, select
``PHY_INTERFACE_RMII`` when the reference clock is supplied to the PHY,
or ``PHY_INTERFACE_RMII_PHY_CLK_OUT`` when the PHY supplies the clock.
The interface mode determines clock direction in both the MAC and PHY
drivers; no separate clock flag or driver-private configuration is needed.
The RTL8201F driver preserves RMII timing and oscillator settings.
RTL8211F programs both delay
bits according to ``PHY_INTERFACE_RGMII``, ``RGMII_ID``, ``RGMII_TXID`` or
``RGMII_RXID`` (no internal delay, both, TX only or RX only). Both drivers
use generic Clause 22 capability discovery, auto-negotiation, negotiated
link-status resolution and power control. Register-page operations are
serialized by the PHYLIB paged-access helpers. These drivers use polling
and auto-negotiation; forced link modes and PHY interrupts are not
implemented.
