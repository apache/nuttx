.. _phylib_vendor_porting:

.. include:: /substitutions.rst

==========================================
PHYLIB Vendor Porting Guide
==========================================

This guide is for vendors implementing a native NuttX Ethernet lower-half.
It explains how to integrate the MDIO upper-half, the PHYLIB lifecycle and a
dedicated PHY model driver. A vendor must ultimately deliver two parts: the
**lower-half PHYLIB integration** and the **dedicated PHY driver**.

1. Scope and Boundaries
=======================

What the vendor must do:

- Keep the MDIO/PHYLIB objects in the driver private struct.
- Implement ``struct mdio_ops_s``.
- Implement and register a dedicated ``struct phy_driver_s``.
- Hook the PHYLIB lifecycle into ifup/ifdown.
- Provide the PHY address, interface, reset and IRQ mode.
- Implement complete error rollback.

Out of scope for this integration:

- Network packet TX/RX algorithms.
- MAC DMA descriptor implementation.
- PTP, QoS and other data-plane TSN capabilities.
- Middleware-specific adaptation.

2. Layering and Call Relationships
==================================

.. note::

   Responsibility boundary: the lower-half handles "how to access the PHY,
   when to create/start/destroy it"; the dedicated PHY driver handles "how
   this model initializes, negotiates, reads status and handles events";
   NuttX handles the state machine, driver matching, standard ioctl,
   carrier and MDIO bus mutual exclusion.

3. The Two Mandatory Vendor Modules
===================================

.. list-table::
   :header-rows: 1
   :widths: 25 40 35

   * - Module
     - Must implement
     - Core responsibility
   * - Ethernet lower-half integration
     - ``mdio_lowerhalf_s``, ``mdio_bus_s *``, ``phy_device_s *``,
       ``mdio_ops_s``, PHYLIB lifecycle
     - Provide MDIO transport; create, connect, start and destroy the PHY.
   * - Dedicated PHY model driver
     - ``phy_driver_s``, identity fields, base callbacks, register API,
       build items
     - Implement the behavior of the new PHY model.

3.1 Lower-half Interface Checklist
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 35 45

   * - Category
     - Interface
     - Requirement
   * - MDIO C22
     - ``mdio_ops_s.read/write``
     - **Mandatory**; return OK on success, negative errno on failure.
   * - PHY reset
     - ``mdio_ops_s.reset``
     - **Implement when a controllable reset line exists**; non-zero
       asserts, zero deasserts.
   * - Native C45
     - ``read_c45/write_c45``
     - **Implement when the hardware supports it**; otherwise C22 indirect
       can be used.
   * - PHY chip registration
     - ``model_register()``
     - Must be before ``phy_device_create()`` and called only once at boot.
   * - ifup
     - ``mdio_register/reset/create/connect/start``
     - Establish and start PHYLIB.
   * - ifdown
     - ``phy_disconnect/mdio_unregister``
     - Stop events first, then free the PHY and the MDIO bus.
   * - PHY event
     - ``phy_interrupt()``
     - Used for ISR-safe notification when the vendor owns the event source.

3.2 Dedicated PHY Driver Interface Checklist
--------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Type
     - Vendor must provide
   * - Identity
     - ``name``, ``phy_id``, ``phy_id_mask``
   * - Base behavior
     - ``config_init``, ``config_aneg``, ``read_status``
   * - By chip capability
     - ``config_power``, ``config_intr``, ``handle_interrupt``,
       ``read_page``, ``write_page``
   * - Registration
     - ``<model>_register()`` and a public header declaration
   * - Build
     - Kconfig, CMakeLists.txt, Make.defs
   * - Optional resources
     - firmware/parameter files, ``phydev->priv`` teardown convention

4. File Organization and Key Structures
=======================================

.. code-block:: text

   vendor/<vendor>/.../<ethernet_lowerhalf>.c
   |-- struct vendor_eth_dev_s
   |-- vendor_mdio_read/write/reset
   |-- g_vendor_mdio_ops
   |-- vendor_phy_setup_once
   |-- vendor_phylib_ifup
   |-- vendor_phylib_ifdown
   `-- vendor_phy_event

   nuttx/drivers/net/phy/<phy_vendor>/
   |-- <model>.c
   |-- <model>.h
   `-- <model>_fw.c/.h          # as needed

   nuttx/include/nuttx/net/<phy_vendor>_phy.h
   nuttx/drivers/net/Kconfig
   nuttx/drivers/net/CMakeLists.txt
   nuttx/drivers/net/Make.defs

The lower-half private struct must at least add:

.. code-block:: c

   struct vendor_eth_dev_s
   {
     struct netdev_lowerhalf_s dev;

     /* Existing Vendor Ethernet state ... */

   #ifdef CONFIG_NET_PHYLIB
     struct mdio_lowerhalf_s   mdio_lower;
     FAR struct mdio_bus_s    *mdio;
     FAR struct phy_device_s  *phydev;

     struct work_s             phy_event_work; /* optional, per event mode */
     uint8_t                   phy_addr;
     enum phy_interface_e      phy_interface;
     int                       phy_irq;
     bool                      phy_ready;
   #endif
   };

A successful ``phy_connect(phydev, &priv->dev)`` also sets
``priv->dev.phydev``, so the netdev upper-half dispatches standard PHY
ioctls to ``phy_ioctl()``.

For switch ports, or MACs that must apply their link mode before reporting
carrier, ``phy_connect(phydev, NULL)`` creates a connection without automatic
carrier notification or netdev ioctl dispatch. The MAC consumes
``state/link/speed/duplex`` under ``phydev->lock``, applies its hardware
configuration, and reports carrier itself. It must forward PHY ioctls
explicitly and release the device with ``phy_disconnect()``. This also
allows several independently negotiated switch PHYs to contribute to one
network interface's carrier without overwriting ``lower->phydev``.

Set ``phydev->interface`` before connecting. For RMII, select
``PHY_INTERFACE_RMII`` when the reference clock is supplied to the PHY,
or ``PHY_INTERFACE_RMII_PHY_CLK_OUT`` when the PHY supplies the clock.
MAC and PHY drivers must handle both modes as RMII signaling, with opposite
reference-clock directions. The PHY driver owns ``phydev->priv``; MAC and
board code must not overwrite or free it.

5. Implementing the MDIO Lower-half
===================================

5.1 Clause 22 Read/Write (mandatory)
------------------------------------

.. code-block:: c

   static int vendor_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                               uint8_t phyaddr, uint8_t regaddr,
                               FAR uint16_t *value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     return vendor_hw_mdio_read(priv, phyaddr, regaddr, value) == 0
            ? OK : -EIO;
   }

   static int vendor_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, uint8_t regaddr,
                                uint16_t value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     return vendor_hw_mdio_write(priv, phyaddr, regaddr, value) == 0
            ? OK : -EIO;
   }

Interface contract:

- ``read/write`` return ``OK`` on success, negative errno on failure.
- ``read`` returns the register value via ``uint16_t *value``.
- Must support a 5-bit PHY address and Clause 22 register addresses.
- The MDIO upper-half already holds the bus mutex before calling the
  callback; the callback must not take that lock again.
- When the hardware API has no independent error code, the vendor must
  define a reliable timeout / no-response check and must not
  unconditionally return success.

5.2 PHY Reset (when a controllable reset line exists)
-----------------------------------------------------

.. code-block:: c

   static int vendor_mdio_reset(FAR struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, int value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     if (value != 0)
       {
         vendor_hw_phy_reset(priv, phyaddr, true);
         up_mdelay(VENDOR_PHY_RESET_ASSERT_MS);
       }
     else
       {
         vendor_hw_phy_reset(priv, phyaddr, false);
         up_mdelay(VENDOR_PHY_RESET_SETTLE_MS);
       }

     return OK;
   }

- ``value != 0``: assert reset.
- ``value == 0``: deassert reset.
- Reset polarity is handled inside the vendor callback.
- The wait after assert and deassert must come from the PHY datasheet.
- With no controllable reset line, leave ``.reset`` NULL; ``mdio_reset()``
  returns ``-ENOSYS``.

5.3 Clause 45 (by hardware capability)
--------------------------------------

Implement when the hardware supports native Clause 45:

.. code-block:: c

   static int vendor_mdio_read_c45(FAR struct mdio_lowerhalf_s *lower,
                                   uint8_t phyaddr, uint8_t devad,
                                   uint16_t regaddr,
                                   FAR uint16_t *value);

   static int vendor_mdio_write_c45(FAR struct mdio_lowerhalf_s *lower,
                                    uint8_t phyaddr, uint8_t devad,
                                    uint16_t regaddr, uint16_t value);

When no native C45 callback is provided, the NuttX MDIO upper-half can use
Clause 22 registers 13/14 for indirect MMD access.

5.4 The ``mdio_ops_s`` Table
----------------------------

.. code-block:: c

   static const struct mdio_ops_s g_vendor_mdio_ops =
   {
     .read  = vendor_mdio_read,
     .write = vendor_mdio_write,
   #ifdef CONFIG_MDIO_CLAUSE_45
     .read_c45  = vendor_mdio_read_c45,  /* as needed */
     .write_c45 = vendor_mdio_write_c45,
   #endif
     .reset = vendor_mdio_reset,         /* as needed */
   };

.. note::

   An MDIO callback must not call an API that re-acquires the same bus
   lock. When an atomic multi-transaction sequence is needed, use
   ``mdio_lock()`` together with ``mdio_*_nolock()``.

6. The Dedicated PHY Model Driver
=================================

.. note::

   For a new PHY needing dedicated init, status parsing, operating modes,
   firmware, paged registers or interrupt handling, the dedicated PHY model
   driver is the vendor's core deliverable. You must not integrate only the
   MDIO lower-half and leave the model logic in the Ethernet lower-half.

6.1 Define ``struct phy_driver_s``
----------------------------------

.. code-block:: c

   static const struct phy_driver_s g_model_driver =
   {
     /* Identity fields: mandatory */
     .name        = "Vendor MODEL",
     .phy_id      = MODEL_PHY_ID,
     .phy_id_mask = MODEL_PHY_ID_MASK,

     /* Base behavior: mandatory by model capability */
     .config_init = model_config_init,
     .config_aneg = model_config_aneg,
     .read_status = model_read_status,

     /* Implement when the chip supports the capability */
     .config_power     = model_config_power,
     .config_intr      = model_config_intr,
     .handle_interrupt = model_handle_interrupt,
     .read_page        = model_read_page,
     .write_page       = model_write_page,
   };

6.2 Identity Fields
-------------------

.. list-table::
   :header-rows: 1
   :widths: 25 40 35

   * - Field
     - Vendor requirement
     - Acceptance
   * - ``name``
     - Provide a recognizable vendor and model name
     - Boot log can identify the bound driver
   * - ``phy_id``
     - Define the 32-bit ID from PHYID1/PHYID2
     - Matches the hardware-measured value
   * - ``phy_id_mask``
     - Mask only bits allowed to vary (e.g. revision)
     - Does not mismatch other models

Match rule:

.. code-block:: c

   (drv->phy_id & drv->phy_id_mask) ==
   (phydev->phy_id & drv->phy_id_mask)

6.3 Base Callbacks
------------------

.. list-table::
   :header-rows: 1
   :widths: 22 22 38 18

   * - API
     - When called
     - Behavior the vendor must implement
     - Return
   * - ``config_init(phydev)``
     - ``phy_connect()``
     - Check ready; soft reset, firmware/parameter load, interface/timing,
       master/slave, operating mode; fill ``supported/advertising/autoneg``
     - OK or negative errno
   * - ``config_aneg(phydev)``
     - ``phy_start()``, ethtool set/reset
     - If auto-negotiation is supported, configure per ``advertising`` and
       restart; otherwise set a fixed mode
     - OK or negative errno
   * - ``read_status(phydev)``
     - Polling or PHY event work
     - Update ``link/speed/duplex``, update ``lp_advertising`` as needed
     - OK on link down; negative errno only on access failure

Clause 22 drivers can reuse ``genphy_c22_config_init()``,
``genphy_c22_config_aneg()``, ``genphy_c22_read_status()``,
``genphy_c22_config_power()`` and ``genphy_c22_soft_reset()`` from the
internal ``drivers/net/phy/phy.h`` interface. Callback tables can reference
these shared operations directly; forwarding wrappers are unnecessary.
Implementation-only helpers remain static. Clause 45 operations use the
separate ``genphy_c45_*`` naming.

6.4 Capability-dependent Callbacks
----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - PHY feature
     - Must implement
     - Key constraint
   * - Power down/up
     - ``config_power(phydev, enable)``
     - true powers up, false powers down
   * - PHY INTB
     - ``config_intr`` + ``handle_interrupt``
     - Decode and clear all interrupt sources in LPWORK
   * - Vendor page
     - ``read_page`` + ``write_page``
     - Must be implemented as a pair, using ``mdio_*_nolock()`` internally
   * - Clause 45/MMD
     - Use ``phy_*_mmd()`` in callbacks
     - Do not call platform-private MDIO APIs
   * - Firmware/parameter tables
     - ``<model>_fw.c/.h``
     - Separate from driver logic, compiled under the same Kconfig
   * - Cross-callback state
     - ``phydev->priv``
     - PHYLIB does not auto-free it currently; define a teardown

6.5 Register APIs the PHY Driver Uses
-------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Access type
     - API to use
   * - Clause 22 ordinary registers
     - ``phy_read``, ``phy_write``, ``phy_modify``, ``phy_set_bits``,
       ``phy_clear_bits``
   * - Clause 45/MMD
     - ``phy_read_mmd``, ``phy_write_mmd``, ``phy_modify_mmd``
   * - Vendor page
     - ``phy_read_paged``, ``phy_write_paged``, ``phy_modify_paged``
   * - Inside page-select callbacks
     - ``mdio_read_nolock``, ``mdio_write_nolock``

6.6 Registration API
--------------------

.. code-block:: c

   int model_register(void)
   {
     return phy_driver_register(&g_model_driver);
   }

Declared in the public header:

.. code-block:: c

   #ifdef CONFIG_NET_PHY_VENDOR_MODEL
   int model_register(void);
   #endif

.. warning::

   Do not put PHY registers, firmware, status parsing, operating modes or
   INTB source decoding in the Ethernet lower-half. Model logic must belong
   to ``phy_driver_s``.

7. One-time Hardware Prep and Driver Registration
=================================================

After the Ethernet/MDIO controller is initialized, initialize the MDIO
hardware and register the dedicated PHY driver before the first
``phy_device_create()``.

.. code-block:: c

   static int vendor_phy_setup_once(FAR struct vendor_eth_dev_s *priv)
   {
     int ret;

     ret = vendor_hw_mdio_initialize(priv);
     if (ret < 0)
       {
         return ret;
       }

   #ifdef CONFIG_NET_PHY_VENDOR_MODEL
     ret = model_register();
     if (ret < 0)
       {
         return ret;
       }
   #endif

     return OK;
   }

Registration rules:

#. Must be earlier than ``phy_device_create()``.
#. Executed only once at boot.
#. Must not be placed in the repeatedly executed ``ifup()``.
#. Must check the return value; do not create the PHY after a failure.
#. ``phy_driver_register()`` currently does not deduplicate; repeated
   registration exhausts ``CONFIG_NET_PHYLIB_MAX_DRIVERS``.

8. Integrating PHYLIB in the Lower-half ifup
============================================

Standard implementation order:

.. code-block:: c

   static int vendor_phylib_ifup(FAR struct vendor_eth_dev_s *priv)
   {
     uint16_t bmsr;
     int ret;

     priv->phy_ready = false;
     priv->mdio_lower.ops = &g_vendor_mdio_ops;

     priv->mdio = mdio_register(&priv->mdio_lower);
     if (priv->mdio == NULL)
       {
         return -ENOMEM;
       }

     ret = mdio_reset(priv->mdio, priv->phy_addr, 1);
     if (ret < 0 && ret != -ENOSYS)
       {
         goto err_mdio;
       }

     ret = mdio_reset(priv->mdio, priv->phy_addr, 0);
     if (ret < 0 && ret != -ENOSYS)
       {
         goto err_mdio;
       }

     ret = mdio_read(priv->mdio, priv->phy_addr, MII_MSR, &bmsr);
     if (ret < 0 || bmsr == 0xffff)
       {
         ret = -EIO;
         goto err_mdio;
       }

     priv->phydev = phy_device_create(priv->mdio, priv->phy_addr);
     if (priv->phydev == NULL)
       {
         ret = -ENODEV;
         goto err_mdio;
       }

     priv->phydev->interface = priv->phy_interface;

     ret = phy_connect(priv->phydev, &priv->dev);
     if (ret < 0)
       {
         phy_device_free(priv->phydev);
         priv->phydev = NULL;
         goto err_mdio;
       }

     priv->phydev->irq = priv->phy_irq; /* use PHY_POLL for first bring-up */

     ret = phy_start(priv->phydev);
     if (ret < 0)
       {
         goto err_disconnect;
       }

     priv->phy_ready = true;
     return OK;

   err_disconnect:
     phy_disconnect(priv->phydev);
     priv->phydev = NULL;
   err_mdio:
     mdio_unregister(priv->mdio);
     priv->mdio = NULL;
     return ret;
   }

.. note::

   First integration should use ``PHY_POLL``. Verify MDIO, the dedicated
   driver, init, auto-negotiation and status reads first, then add PHY
   interrupts.

9. Standard ifdown, Teardown and Failure Rollback
=================================================

This guide mandates one standard flow: each ``ifup`` creates and starts the
MDIO/PHY objects, and each ``ifdown`` fully destroys them.

.. code-block:: c

   static void vendor_phylib_ifdown(FAR struct vendor_eth_dev_s *priv)
   {
     priv->phy_ready = false;

     /* No callback/work may access phydev after this. */

     vendor_phy_event_disable(priv);
     work_cancel_sync(LPWORK, &priv->phy_event_work);

     if (priv->phydev != NULL)
       {
         /* phy_disconnect() already calls phy_stop() internally. */
         phy_disconnect(priv->phydev);
         priv->phydev = NULL;
       }

     if (priv->mdio != NULL)
       {
         mdio_unregister(priv->mdio);
         priv->mdio = NULL;
       }
   }

.. note::

   You need not call ``phy_stop()`` explicitly first: ``phy_disconnect()``
   already stops the PHY, disables PHY interrupts and synchronously cancels
   the PHYLIB state work. Calling stop before disconnect is redundant.

9.1 Failure Rollback Table
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Failure point
     - Must roll back
   * - ``mdio_register`` failed
     - No bus; return directly
   * - reset/probe/create failed
     - ``mdio_unregister``
   * - create ok, connect failed
     - ``phy_device_free`` -> ``mdio_unregister``
   * - connect ok, start failed
     - ``phy_disconnect`` -> ``mdio_unregister``
   * - Vendor event/work already started
     - Disable the event source, then ``work_cancel_sync``, then free the PHY

.. warning::

   Do not unregister the MDIO bus first. ``phy_disconnect()`` may still use
   that bus to perform stop, power and reset operations.

10. PHY Link Events and Interrupt Modes
=======================================

.. list-table::
   :header-rows: 1
   :widths: 25 30 45

   * - ``phydev->irq``
     - Mode
     - What the vendor lower-half does
   * - ``PHY_POLL (-1)``
     - No interrupt / first bring-up
     - Provide no event ISR; PHYLIB calls ``read_status()`` periodically
   * - ``PHY_MAC_INTERRUPT (-2)``
     - Vendor owns the GPIO/ICU/controller event source
     - The ISR/callout only records the event and calls ``phy_interrupt()``
   * - ``>= 0``
     - A real IRQ attachable by NuttX ``irq_attach()``
     - Fill in the real IRQ; PHYLIB attaches/detaches the ISR

10.1 Vendor Owns the Event Source
---------------------------------

.. code-block:: c

   static void vendor_phy_event(FAR struct vendor_eth_dev_s *priv)
   {
     if (priv->phy_ready && priv->phydev != NULL)
       {
         phy_interrupt(priv->phydev);
       }
   }

``phy_interrupt()`` only posts state-machine work and may be called from
interrupt context; the real MDIO access happens in LPWORK.

10.2 Interrupt Callbacks of the PHY Model Driver
------------------------------------------------

.. code-block:: c

   static int model_config_intr(FAR struct phy_device_s *phydev,
                                bool enable);

   static int model_handle_interrupt(FAR struct phy_device_s *phydev);

``handle_interrupt()`` return values:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Return
     - Meaning
   * - ``> 0``
     - A link-related event occurred; PHYLIB continues to call ``read_status()``
   * - ``0``
     - No link-related event; keep the current link state
   * - ``< 0``
     - MDIO/HW failure; PHYLIB enters ``PHY_ERROR``

.. warning::

   Calling ``mdio_read/write`` in a hard IRQ is forbidden. If INTB is
   level-triggered, ``handle_interrupt()`` must read and clear all active
   PHY interrupt sources before returning, otherwise it causes an interrupt
   storm.

11. ioctl and Carrier
=====================

``phy_connect()`` associates the PHY device with
``netdev_lowerhalf_s.phydev``. The NuttX netdev upper-half then calls first:

.. code-block:: c

   if (lower->phydev != NULL)
     {
       ret = phy_ioctl(lower->phydev, cmd, arg);
       if (ret != -ENOTTY)
         {
           return ret;
         }
     }

PHYLIB already handles:

- ``SIOCGMIIPHY``
- ``SIOCGMIIREG``
- ``SIOCSMIIREG``
- ``ETHTOOL_GLINKSETTINGS``
- ``ETHTOOL_SLINKSETTINGS``
- ``ETHTOOL_NWAY_RST``
- ``ETHTOOL_GLINK``

Therefore:

- The vendor lower-half usually should not re-handle the standard MII
  ioctls.
- Only private commands for which PHYLIB returns ``-ENOTTY`` continue to
  the lower-half.
- The PHYLIB state machine calls ``netdev_lower_carrier_on/off()`` on link
  change.
- The vendor should not run another standard link-polling loop or
  repeatedly modify ``phydev->link`` or the carrier.

12. Kconfig and Build Integration
==================================

12.1 Base Configuration
-----------------------

.. code-block:: kconfig

   CONFIG_NET=y
   CONFIG_NETDEVICES=y
   CONFIG_MDIO_BUS=y
   CONFIG_NET_PHYLIB=y
   CONFIG_NET_PHYLIB_POLL_INTERVAL=1000

``NET_PHYLIB`` currently depends on but does not auto-select
``MDIO_BUS``. Enable Clause 45/MMD as needed:

.. code-block:: kconfig

   CONFIG_MDIO_CLAUSE_45=y

12.2 Dedicated PHY Model Kconfig
--------------------------------

.. code-block:: kconfig

   config NET_PHY_VENDOR_MODEL
     bool "Vendor MODEL Ethernet PHY"
     default n
     depends on NET_PHYLIB
     ---help---
       Enable support for the Vendor MODEL Ethernet PHY.

12.3 CMakeLists.txt
-------------------

.. code-block:: cmake

   if(CONFIG_NET_PHY_VENDOR_MODEL)
     list(APPEND SRCS phy/<phy_vendor>/<model>.c)

     # When the model needs firmware/parameter objects:
     # list(APPEND SRCS phy/<phy_vendor>/<model>_fw.c)
   endif()

12.4 Make.defs
--------------

.. code-block:: makefile

   ifeq ($(CONFIG_NET_PHY_VENDOR_MODEL),y)
     CSRCS += <phy_vendor>/<model>.c
     # CSRCS += <phy_vendor>/<model>_fw.c
   endif

13. Vendor From-scratch SOP
===========================

#. **Confirm the lower-half**: the private struct has
   ``netdev_lowerhalf_s``, and ifup/ifdown can integrate PHYLIB.
#. **Prepare MDIO hardware**: init pins, clock and controller; verify you
   can read BMSR and PHYID1/PHYID2.
#. **Embed the three objects**: ``mdio_lower``, ``mdio``, ``phydev``.
#. **Implement mdio_ops_s**: C22 read/write mandatory; reset/C45 as the
   hardware provides.
#. **Implement the dedicated PHY driver**: define ``phy_driver_s``,
   implement identity fields, init/aneg/status and applicable advanced
   callbacks.
#. **Add registration and build**: provide the public registration
   declaration; edit Kconfig, CMakeLists.txt, Make.defs.
#. **Register once**: call ``model_register()`` before
   ``phy_device_create()``, only once at boot.
#. **Hook ifup**: register -> reset -> probe -> create -> interface ->
   connect -> IRQ -> start.
#. **Hook ifdown**: stop event/work -> disconnect -> unregister.
#. **Remove duplicate management**: leave standard MII ioctl, link polling
   and carrier to PHYLIB.
#. **Polling first, interrupt later**: validate init/aneg/status first,
   then the interrupt callbacks.

14. Two Ready-to-adapt Templates
================================

.. note::

   In the templates, ``vendor_hw_*``, the PHY address, interface, IRQ and
   register macros are the vendor porting points. After replacing these
   hardware hooks, add C45, page, INTB, power and firmware per the actual
   chip capability.

14.1 Vendor Ethernet Lower-half PHYLIB Template
-----------------------------------------------

.. code-block:: c

   #include <nuttx/config.h>

   #include <errno.h>
   #include <nuttx/arch.h>
   #include <nuttx/net/mdio.h>
   #include <nuttx/net/mii.h>
   #include <nuttx/net/netdev_lowerhalf.h>
   #include <nuttx/net/phy.h>
   #include <nuttx/net/<phy_vendor>_phy.h>
   #include <nuttx/wqueue.h>

   struct vendor_eth_dev_s
   {
     struct netdev_lowerhalf_s dev;

     /* Existing Vendor Ethernet state ... */

   #ifdef CONFIG_NET_PHYLIB
     struct mdio_lowerhalf_s   mdio_lower;
     FAR struct mdio_bus_s    *mdio;
     FAR struct phy_device_s  *phydev;
     struct work_s             phy_event_work;
     uint8_t                   phy_addr;
     enum phy_interface_e      phy_interface;
     int                       phy_irq;
     bool                      phy_ready;
   #endif
   };

   static int vendor_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                               uint8_t phyaddr, uint8_t regaddr,
                               FAR uint16_t *value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     return vendor_hw_mdio_read(priv, phyaddr, regaddr, value) == 0
            ? OK : -EIO;
   }

   static int vendor_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, uint8_t regaddr,
                                uint16_t value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     return vendor_hw_mdio_write(priv, phyaddr, regaddr, value) == 0
            ? OK : -EIO;
   }

   static int vendor_mdio_reset(FAR struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, int value)
   {
     FAR struct vendor_eth_dev_s *priv =
       container_of(lower, struct vendor_eth_dev_s, mdio_lower);

     if (value != 0)
       {
         vendor_hw_phy_reset(priv, phyaddr, true);
         up_mdelay(VENDOR_PHY_RESET_ASSERT_MS);
       }
     else
       {
         vendor_hw_phy_reset(priv, phyaddr, false);
         up_mdelay(VENDOR_PHY_RESET_SETTLE_MS);
       }

     return OK;
   }

   static const struct mdio_ops_s g_vendor_mdio_ops =
   {
     .read  = vendor_mdio_read,
     .write = vendor_mdio_write,
     .reset = vendor_mdio_reset,
   };

   static int vendor_phy_setup_once(FAR struct vendor_eth_dev_s *priv)
   {
     int ret;

     ret = vendor_hw_mdio_initialize(priv);
     if (ret < 0)
       {
         return ret;
       }

   #ifdef CONFIG_NET_PHY_VENDOR_MODEL
     ret = model_register();
     if (ret < 0)
       {
         return ret;
       }
   #endif

     return OK;
   }

   static int vendor_phylib_ifup(FAR struct vendor_eth_dev_s *priv)
   {
     uint16_t bmsr;
     int ret;

     priv->phy_ready = false;
     priv->mdio_lower.ops = &g_vendor_mdio_ops;

     priv->mdio = mdio_register(&priv->mdio_lower);
     if (priv->mdio == NULL)
       {
         return -ENOMEM;
       }

     ret = mdio_reset(priv->mdio, priv->phy_addr, 1);
     if (ret < 0 && ret != -ENOSYS)
       {
         goto err_mdio;
       }

     ret = mdio_reset(priv->mdio, priv->phy_addr, 0);
     if (ret < 0 && ret != -ENOSYS)
       {
         goto err_mdio;
       }

     ret = mdio_read(priv->mdio, priv->phy_addr, MII_MSR, &bmsr);
     if (ret < 0 || bmsr == 0xffff)
       {
         ret = -EIO;
         goto err_mdio;
       }

     priv->phydev = phy_device_create(priv->mdio, priv->phy_addr);
     if (priv->phydev == NULL)
       {
         ret = -ENODEV;
         goto err_mdio;
       }

     priv->phydev->interface = priv->phy_interface;

     ret = phy_connect(priv->phydev, &priv->dev);
     if (ret < 0)
       {
         phy_device_free(priv->phydev);
         priv->phydev = NULL;
         goto err_mdio;
       }

     priv->phydev->irq = priv->phy_irq;
     ret = phy_start(priv->phydev);
     if (ret < 0)
       {
         goto err_disconnect;
       }

     priv->phy_ready = true;
     return OK;

   err_disconnect:
     phy_disconnect(priv->phydev);
     priv->phydev = NULL;
   err_mdio:
     mdio_unregister(priv->mdio);
     priv->mdio = NULL;
     return ret;
   }

   static void vendor_phylib_ifdown(FAR struct vendor_eth_dev_s *priv)
   {
     priv->phy_ready = false;
     vendor_phy_event_disable(priv);
     work_cancel_sync(LPWORK, &priv->phy_event_work);

     if (priv->phydev != NULL)
       {
         phy_disconnect(priv->phydev);
         priv->phydev = NULL;
       }

     if (priv->mdio != NULL)
       {
         mdio_unregister(priv->mdio);
         priv->mdio = NULL;
       }
   }

   static void vendor_phy_event(FAR struct vendor_eth_dev_s *priv)
   {
     if (priv->phy_ready && priv->phydev != NULL)
       {
         phy_interrupt(priv->phydev);
       }
   }

14.2 Dedicated PHY Model Driver Template
----------------------------------------

.. code-block:: c

   /* drivers/net/phy/<phy_vendor>/<model>.c */
   #include <nuttx/config.h>

   #include <errno.h>
   #include <nuttx/ethtool.h>
   #include <nuttx/net/mii.h>
   #include <nuttx/net/phy.h>
   #include <nuttx/net/<phy_vendor>_phy.h>

   #include "phy.h"
   #include "<model>.h"

   static int model_config_init(FAR struct phy_device_s *phydev);
   static int model_config_aneg(FAR struct phy_device_s *phydev);
   static int model_read_status(FAR struct phy_device_s *phydev);

   static const struct phy_driver_s g_model_driver =
   {
     .name        = "Vendor MODEL",
     .phy_id      = MODEL_PHY_ID,
     .phy_id_mask = MODEL_PHY_ID_MASK,
     .config_init = model_config_init,
     .config_aneg = model_config_aneg,
     .read_status = model_read_status,
   };

   static int model_config_init(FAR struct phy_device_s *phydev)
   {
     int ret;

     ret = model_soft_reset(phydev); /* implement per datasheet */
     if (ret < 0)
       {
         return ret;
       }

     phydev->supported = SUPPORTED_10BASET_FULL |
                         SUPPORTED_100BASET_FULL |
                         SUPPORTED_AUTONEG |
                         SUPPORTED_TP |
                         SUPPORTED_MII;
     phydev->advertising = phydev->supported;
     phydev->autoneg = true;
     return OK;
   }

   static int model_config_aneg(FAR struct phy_device_s *phydev)
   {
     uint16_t adv = MII_ADVERTISE_CSMA;
     int ret;

     if (phydev->advertising & SUPPORTED_10BASET_FULL)
       {
         adv |= MII_ADVERTISE_10BASETXFULL;
       }

     if (phydev->advertising & SUPPORTED_100BASET_FULL)
       {
         adv |= MII_ADVERTISE_100BASETXFULL;
       }

     ret = phy_write(phydev, MII_ADVERTISE, adv);
     if (ret < 0)
       {
         return ret;
       }

     return phy_modify(phydev, MII_MCR,
                       MII_MCR_ANENABLE | MII_MCR_ANRESTART,
                       MII_MCR_ANENABLE | MII_MCR_ANRESTART);
   }

   static int model_read_status(FAR struct phy_device_s *phydev)
   {
     int status;

     status = phy_read(phydev, MODEL_STATUS_REG);
     if (status < 0 || status == 0xffff)
       {
         return -EIO;
       }

     /* Read twice only when the status bit is latch-low. */
     status = phy_read(phydev, MODEL_STATUS_REG);
     if (status < 0 || status == 0xffff)
       {
         return -EIO;
       }

     phydev->link = (status & MODEL_STATUS_LINK) != 0;
     if (!phydev->link)
       {
         phydev->speed = 0;
         phydev->duplex = DUPLEX_UNKNOWN;
         return OK;
       }

     return model_decode_speed_duplex(phydev, (uint16_t)status);
   }

   int model_register(void)
   {
     return phy_driver_register(&g_model_driver);
   }

If the chip needs power, INTB, page, MMD or firmware, add the corresponding
callbacks from the Section 6 decision tables to ``g_model_driver``; do not
move that model logic back to the lower-half.

15. Mechanized Delivery Checklist (Two Patches)
===============================================

15.1 Patch 1: Dedicated PHY Model Driver
----------------------------------------

.. code-block:: text

   A  drivers/net/phy/<phy_vendor>/<model>.c
   A  drivers/net/phy/<phy_vendor>/<model>.h
   A  include/nuttx/net/<phy_vendor>_phy.h
   A  drivers/net/phy/<phy_vendor>/<model>_fw.c/.h  # as needed
   M  drivers/net/Kconfig
   M  drivers/net/CMakeLists.txt
   M  drivers/net/Make.defs

Checklist:

- ``phy_driver_s`` identity fields and callbacks complete.
- All registers, bits and timing have a datasheet basis.
- ``model_register()`` has a public declaration.
- Firmware has source, version and license notes.
- Standalone testing of the dedicated driver is done.

15.2 Patch 2: Vendor Ethernet Lower-half PHYLIB Integration
-----------------------------------------------------------

.. code-block:: text

   M  <vendor Ethernet lower-half>.c
   M  <vendor Ethernet config/header>
   M  <board/project defconfig>

Checklist:

- The private struct contains the three PHYLIB/MDIO members.
- ``mdio_ops_s`` read/write/reset/C45 implemented by capability.
- ``model_register()`` has a one-time call site.
- ifup creates and starts in the standard order.
- ifdown stops events and destroys in the standard order.
- PHY event delivery is correct.
- Every failure point rolls back symmetrically.

.. note::

   Patch 1 is a PHY model driver reusable across Ethernet controllers;
   Patch 2 contains only this vendor lower-half's MDIO transport, PHYLIB
   lifecycle and board-level inputs.

16. Reference Implementation Mapping
====================================

.. note::

   The reference implementation confirms how the interfaces land. Reuse
   only the NuttX PHYLIB pattern; do not copy platform registers, GPIO,
   interrupt controllers, cross-core sync, DFX, recovery strategies or
   another PHY's registers/firmware.

Native NuttX Ethernet lower-half reference:
``vendor/infineon/chips/aurix/ethernet/aurix_leth.c``

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Reference code
     - Generic requirement
     - New vendor driver should implement
   * - ``struct aurix_leth_dev_s``
     - Hold PHYLIB objects in the netdev private state
     - ``mdio_lowerhalf_s``, ``mdio_bus_s *``, ``phy_device_s *``, event work
       as needed
   * - ``phylib_mdio_read/write``
     - Clause 22 MDIO lower-half
     - Recover the instance with ``container_of``, forward to hardware,
       return OK/negative errno
   * - ``phylib_mdio_reset``
     - PHY reset level callback
     - Non-zero asserts, zero deasserts; timing from the datasheet
   * - ``g_phylib_mdio_ops``
     - MDIO lower-half vtable
     - C22 mandatory; reset/C45 by capability
   * - ``aurix_phy_setup``
     - One-time hardware prep and model registration
     - Init the MDIO controller, then ``model_register()`` once
   * - ``aurix_phylib_ifup``
     - PHYLIB establish and start
     - register -> reset -> probe -> create -> connect -> IRQ -> start
   * - ``aurix_leth_ifdown``
     - PHYLIB stop and destroy
     - Stop event/work first, then disconnect -> unregister
   * - ``phy_intb_callout``
     - Platform owns the PHY event source
     - Latch/defer only, finally call ``phy_interrupt()``

Reference notes:

#. ``phy_disconnect()`` already runs ``phy_stop()``; the generic template
   does not repeat it.
#. ``model_register()`` and ``mdio_reset()`` must check return values.
#. Standard MII ioctl is already forwarded to ``phy_ioctl()`` by the netdev
   upper-half; the lower-half should not re-implement it.
#. A platform link worker must not bypass PHYLIB to modify ``phydev->link``
   directly.
#. GPIO, ICU, middleware, cross-core shared memory, platform-specific
   recovery and diagnostics are not generic requirements.

Dedicated PHY model driver reference:
``drivers/net/phy/realtek/rtl9000b.c``

The in-tree RTL9000B driver (an automotive 100BASE-T1 PHY) is a complete
example of a dedicated PHY model driver:

- A complete ``phy_driver_s`` covering ``config_init`` / ``config_aneg`` /
  ``read_status`` / ``config_power`` / ``config_intr`` /
  ``handle_interrupt`` / ``read_page`` / ``write_page``.
- ``config_init`` does the ready check, parameter write/verify, voltage
  config, soft reset, operating-mode selection and capability publication.
- ``read_status`` updates ``link``/``speed``/``duplex`` and returns OK on a
  normal link down.
- ``handle_interrupt`` decodes link / under-voltage / over-temperature and
  recovery events and reports whether the link must be re-read.
- ``rtl9000b_register()`` wraps ``phy_driver_register(&g_rtl9000b_driver)``.
- The chip parameter tables live in a separate firmware file
  (``rtl9000b_fw.c``), compiled conditionally by Kconfig.

17. Acceptance Gate
===================

Lower-half integration:

- Contains ``mdio_lower/mdio/phydev``.
- MDIO callbacks return OK/negative errno.
- ifup lifecycle order is correct.
- Every failure point releases resources symmetrically.
- ifdown stops events and work first.
- Disconnect happens before unregistering the bus.
- Standard MII ioctl is not re-implemented.

Dedicated PHY driver:

- ``phy_driver_s`` identity fields complete.
- ID/mask matches only the target model.
- ``config_init`` completes all model initialization.
- ``config_aneg`` matches the product mode.
- ``read_status`` correctly updates the three status items.
- Applicable power/interrupt/page callbacks implemented.
- Register declaration and the three build entries complete.
- No model logic remains in the lower-half.

Joint function and stability:

- Boot binds the dedicated driver, not the generic PHY.
- Carrier off with no cable.
- Carrier on after plugging; speed/duplex correct.
- Unplug and re-plug work correctly.
- 100 ifup/ifdown cycles.
- MII ioctl and ethtool work.
- No worker/IRQ/UAF/MDIO bus leaks.
- No storm or missed events in interrupt mode.

18. Common Mistakes
===================

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Symptom
     - Cause and fix
   * - PHY ID is ``0xffff/0x0000``
     - Check the MDIO controller, address, reset, clock and pins first
   * - Matches the generic PHY
     - Model not registered, registered too late, not compiled, or wrong
       ID/mask
   * - ``-ENOMEM`` after repeated registration
     - ``model_register()`` placed in the repeated ifup; move it to the
       one-time setup
   * - Leak after connect failure
     - Missing ``phy_device_free()``
   * - ifdown / UAF
     - Event source not disabled or work not synchronously cancelled before
       freeing the PHY
   * - Paged access deadlock
     - Page callback wrongly uses the locking ``mdio_read/write``; use
       ``_nolock``
   * - Standard MII ioctl re-implemented
     - Should be forwarded to ``phy_ioctl()`` by the netdev upper-half
   * - Interrupt storm
     - ``handle_interrupt()`` did not clear the PHY internal level source
   * - PHY state correct but no data
     - Keep checking the Ethernet data plane and controller config; do not
       blame PHYLIB directly

19. Current Source References
=============================

.. code-block:: text

   nuttx/include/nuttx/net/netdev_lowerhalf.h
   nuttx/include/nuttx/net/mdio.h
   nuttx/include/nuttx/net/phy.h
   nuttx/drivers/net/mdio.c
   nuttx/drivers/net/phy/phy.h
   nuttx/drivers/net/phy/phy_device.c
   nuttx/drivers/net/phy/phy_core.c
   nuttx/drivers/net/phy/phy_access.c
   nuttx/drivers/net/phy/phy_gen.c
   nuttx/drivers/net/phy/phy_genc45.c
   nuttx/drivers/net/Kconfig
   nuttx/drivers/net/CMakeLists.txt
   nuttx/drivers/net/Make.defs
