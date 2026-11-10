.. _phylib:

.. include:: /substitutions.rst

==========================================
PHY Abstraction Layer (PHYLIB)
==========================================

This document describes the design of the PHY Abstraction Layer (PHYLIB)
for NuttX. It covers: design background, overall architecture, core data
structures, the state machine, the generic PHY driver, MAC driver
integration, ethtool integration, vendor drivers, and debugging.

1. Design Background
====================

1.1 Current Problems
--------------------

In the current NuttX network drivers, PHY management logic is scattered
inside each MAC driver. Drawbacks of having no PHYLIB:

#. **Large amount of duplicated code** — every MAC driver implements its
   own MDIO read/write, PHY init and link detection; the logic is similar
   but rewritten each time.
#. **Changing the PHY means changing the MAC** — PHY register definitions
   are spread across MAC drivers, so swapping a chip requires editing MAC
   source.
#. **Inconsistent link management** — each driver uses a different poll
   interval, and carrier notification timing is not unified.
#. **PHY drivers cannot be reused** — the same PHY on a different SoC has
   its init code copied again.
#. **Inconsistent ioctl behavior** — MII/ethtool commands are interpreted
   by each driver on its own.

1.2 Design Goals
----------------

#. Decouple PHY management from the MAC driver.
#. Provide a standardized MDIO bus, PHY device and PHY driver framework.
#. Support Clause 22 and Clause 45 PHYs.
#. Support both polling and interrupt link-monitoring modes, selectable
   by hardware capability.
#. Expose link state uniformly through ethtool ioctl.
#. Allow PHY drivers to be reused across platforms.

.. note::

   Core value: a MAC driver only needs to implement the MDIO lower-half
   (the ``mdio_ops_s`` read/write callbacks) and call a few PHYLIB APIs to
   complete PHY integration, without caring about the register details of
   a specific PHY chip. The MDIO bus itself is managed by the community
   upper-half framework (``drivers/net/mdio.c``).

1.3 Provenance
--------------

This implementation is written independently based on the IEEE 802.3
standard and chip datasheets (such as the RTL8221B). Register addresses,
bit definitions and access timing are IEEE standard / hardware facts.
The interface naming follows common PHYLIB conventions for readability.
The underlying implementation (state machine, concurrency model, memory
management) is a native NuttX design.

2. Overall Architecture
=======================

The framework follows the ``netdev -> phydev -> mdio`` layering. Benefits
of PHYLIB:

#. **Zero change to swap a PHY** — only add a vendor driver file; the MAC
   driver is untouched.
#. **Unified state machine** — one piece of logic serves all MACs; link
   changes are reported to the stack automatically.
#. **PHY drivers reusable across platforms** — a vendor driver works on any
   MAC platform.
#. **Fast bring-up on new platforms** — implement the MDIO lower-half
   (``mdio_ops_s``) and call a few PHYLIB APIs.
#. **Standardized ioctl** — PHYLIB handles MII/ethtool commands uniformly.
#. **Polling / interrupt dual mode** — polling by default; if the hardware
   wires INTB, switching to interrupt-driven is free.

3. Core Source Files
====================

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - File
     - Role
   * - ``drivers/net/phy/phy.c``
     - State machine: start/stop/poll worker, interrupt handling
       (``phy_mac_interrupt``), suspend/resume.
   * - ``drivers/net/phy/phy_device.c``
     - PHY device model: create, match, connect, scan.
   * - ``drivers/net/mdio.c``
     - MDIO bus upper-half (community framework): ``mdio_register``,
       ``mdio_read``/``write``, ``mdio_c45`` access, serialized by a lock.
   * - ``drivers/net/phy/phy_core.c``
     - Low-level helpers: ``phy_read``/``write``/``modify``/``set_bits``/``mmd``.
   * - ``drivers/net/phy/phy_ioctl.c``
     - MII ioctl + ethtool link-settings dispatch.
   * - ``drivers/net/phy/genphy.c``
     - Generic Clause 22 PHY driver (IEEE 802.3 standard registers).
   * - ``drivers/net/phy/genphy_c45.c``
     - Generic Clause 45 PHY driver (MMD registers).
   * - ``drivers/net/phy/realtek/``
     - RTL8221B vendor driver.
   * - ``include/nuttx/net/phy.h``
     - Public header: structures + API declarations.
   * - ``include/nuttx/net/mdio.h``
     - MDIO upper/lower-half contract: ``mdio_ops_s``,
       ``mdio_lowerhalf_s``, ``mdio_register`` etc.

4. Kconfig
==========

.. code-block:: kconfig

   # Framework core
   CONFIG_NET_PHYLIB=y                    # Enable the PHYLIB subsystem
                                          # Auto-selects the two options below:
   CONFIG_MDIO_BUS=y                      # MDIO bus upper-half (drivers/net/mdio.c)
   CONFIG_MDIO_CLAUSE_45=y                # MDIO Clause 45 support (needed by C45 PHYs)

   # Tunables
   CONFIG_NET_PHYLIB_POLL_INTERVAL=1000   # State-machine poll interval (ms)
   CONFIG_NET_PHYLIB_MAX_DRIVERS=8        # Maximum number of PHY drivers
   CONFIG_NET_PHYLIB_MAX_PHYS=8           # Maximum number of PHY devices

5. Core Data Structures
=======================

5.1 MDIO Bus
------------

The MDIO bus uses the standard upper/lower-half layering
(``drivers/net/mdio.c`` + ``include/nuttx/net/mdio.h``). The upper-half
owns the bus core object (lock, read/write forwarding, lifecycle); the
MAC driver only implements the lower-half hardware callbacks.

Design points:

- **Upper/lower layering**: the bus core object (including the lock) lives
  in the upper-half ``drivers/net/mdio.c`` and is allocated dynamically by
  ``mdio_register()``. The MAC driver only holds a ``mdio_lowerhalf_s`` and
  the handle returned by ``mdio_register()``.
- **Locking**: the read/write mutex is held by the upper-half, guaranteeing
  concurrency safety between ethtool ioctl and the state machine.
- **C45**: ``read_c45``/``write_c45`` are optional — when NULL, the
  upper-half automatically uses the Clause 22 reg 13/14 indirect access;
  implementing them when the hardware natively supports C45 (e.g. AURIX
  GETH) avoids the indirect transactions. Controlled by
  ``CONFIG_MDIO_CLAUSE_45`` (see "C45 Extension" below).

Lower-half callback table ``mdio_ops_s`` (implemented by the MAC driver):

.. code-block:: c

   struct mdio_ops_s
   {
     /* Clause 22 read/write */
     int (*read)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                 uint8_t regaddr, FAR uint16_t *value);
     int (*write)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                  uint8_t regaddr, uint16_t value);

   #ifdef CONFIG_MDIO_CLAUSE_45
     /* Native Clause 45 read/write - optional; when NULL the upper-half
      * uses reg13/14 indirect access.
      */
     int (*read_c45)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                     uint8_t devad, uint16_t regaddr, FAR uint16_t *value);
     int (*write_c45)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                      uint8_t devad, uint16_t regaddr, uint16_t value);
   #endif

     /* PHY hardware reset line control - optional. Controls the PHY RESET_N
      * by logic level rather than a one-shot pulse. value != 0 asserts
      * (into reset); value == 0 deasserts (release). The lower-half handles
      * pin polarity (active-low RESET_N handled internally) and the
      * datasheet-specified assert/deassert settle delays, and any
      * idempotency (a single bus may host several PHY addresses).
      */
     int (*reset)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                  int value);
   };

   /* Lower-half instance: allocated by the MAC driver; ops must be the
    * first member.
    */
   struct mdio_lowerhalf_s
   {
     FAR const struct mdio_ops_s *ops;
   };

Upper-half implementation (``drivers/net/mdio.c``) and public API. The
upper-half collapses the bus core into ``struct mdio_bus_s`` (defined
privately inside mdio.c, not visible externally), holding only a
"lower-half pointer + mutex":

.. code-block:: c

   /* Register the lower-half */
   FAR struct mdio_bus_s *mdio_register(FAR struct mdio_lowerhalf_s *lower);
   int mdio_unregister(FAR struct mdio_bus_s *dev);

   /* Read/write (serialized by the upper-half internal lock) */
   int mdio_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                 uint8_t regaddr, FAR uint16_t *value);
   int mdio_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                  uint8_t regaddr, uint16_t value);

   #ifdef CONFIG_MDIO_CLAUSE_45
   int mdio_c45_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                     uint8_t devad, uint16_t regaddr, FAR uint16_t *value);
   int mdio_c45_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                      uint8_t devad, uint16_t regaddr, uint16_t value);
   #endif

   /* PHY hardware reset line control (assert/deassert) */
   int mdio_reset(FAR struct mdio_bus_s *dev, uint8_t phyaddr, int value);

   struct mdio_bus_s
   {
     FAR struct mdio_lowerhalf_s *lower;
     mutex_t                      lock;        /* Bus mutex */
   };

C45 Extension
~~~~~~~~~~~~~

The original community upper-half implemented only Clause 22 (C45 was left
as an empty ``#ifdef CONFIG_MDIO_CLAUSE_45``). Since the RTL8221B is a C45
PHY, C45 was completed on top of the community framework:

#. **``mdio_ops_s`` gains** two optional callbacks ``read_c45`` /
   ``write_c45`` (see the structure above).
#. **``mdio.c`` gains** ``mdio_c45_read`` / ``mdio_c45_write``: when the
   lower-half provides native C45 ops they are called directly; otherwise
   it **falls back automatically** to the Clause 22 reg13/14 indirect
   access (MMD indirect addressing), transparent to upper layers:

   .. code-block:: c

      int mdio_c45_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                        uint8_t devad, uint16_t regaddr, FAR uint16_t *value)
      {
        nxmutex_lock(&dev->lock);

        if (dev->lower->ops->read_c45 != NULL)
          {
            /* Native C45: one transaction in the lower-half */
            int ret = dev->lower->ops->read_c45(dev->lower, phyaddr, devad,
                                                regaddr, value);
            nxmutex_unlock(&dev->lock);
            return ret;
          }

        /* Fallback: indirect via C22 reg13(MMDCTRL)/reg14(MMDDATA) */
        dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL, devad);
        dev->lower->ops->write(dev->lower, phyaddr, MII_MMDADDRDATA, regaddr);
        dev->lower->ops->write(dev->lower, phyaddr, MII_MMDCONTROL,
                               devad | MII_MMDCONTROL_FUNC_NOINCR);
        int ret = dev->lower->ops->read(dev->lower, phyaddr,
                                        MII_MMDADDRDATA, value);
        nxmutex_unlock(&dev->lock);
        return ret;
      }

#. **Kconfig gains** a ``CONFIG_MDIO_CLAUSE_45`` switch, which
   ``NET_PHYLIB`` auto-selects.

.. note::

   Meaning of the C45 fallback: when the hardware natively supports C45
   (e.g. AURIX GETH), implementing ``read_c45``/``write_c45`` completes the
   access in one bus transaction; a MAC without native C45 need not write
   indirect-access logic itself, as the upper-half falls back to reg13/14
   automatically (4 C22 transactions). The PHYLIB caller
   (``phy_read_mmd`` -> ``mdio_c45_read``) is completely unaware.

reg13/14 indirect addressing (IEEE 802.3 Clause 22 extension, the standard
mechanism for accessing MMD registers): the C22 register space only has a
5-bit address (reg 0-31) and cannot directly address the 16-bit MMD
register space of C45. IEEE 802.3 uses **reg 13 (MMDCONTROL) + reg 14
(MMDADDRDATA)** as an indirect channel, so each C45 access takes 4 steps:

.. list-table::
   :header-rows: 1
   :widths: 10 15 40 35

   * - Step
     - Operation
     - reg 13 ``MMDCONTROL``
     - reg 14 ``MMDADDRDATA``
   * - 1
     - C22 write
     - devad (address mode, FUNC=00)
     - —
   * - 2
     - C22 write
     - —
     - Write the target 16-bit MMD register address
   * - 3
     - C22 write
     - devad | data mode (FUNC=01, no post-increment)
     - —
   * - 4
     - C22 read / write
     - —
     - Read / write the 16-bit data of the target MMD register

Each C45 read/write needs 4 C22 bus transactions, 4x the cost of native
C45 (one frame of 5-bit OP + 16-bit address + 16-bit data). So implementing
``read_c45``/``write_c45`` when the hardware supports native C45 (e.g.
AURIX GETH) saves 75% of the bus transactions, a significant optimization.

Interface provenance (community original / added by this design):

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Interface / field
     - Source
     - Notes
   * - ``mdio_register`` / ``mdio_unregister``
     - community
     - Dynamically allocate/free the bus handle, bind the lower-half.
   * - ``mdio_read`` / ``mdio_write``
     - community
     - Clause 22 read/write, forward to ``lower->ops`` under the lock.
   * - ``mdio_reset``
     - community
     - PHY hardware reset line control: changed to ``(lower, phyaddr,
       value)`` level control; idempotency is left to the lower half.
   * - ``mdio_ops_s.read`` / ``.write`` / ``.reset``
     - community
     - Lower-half C22 + reset callback contract.
   * - ``mdio_ops_s.read_c45`` / ``.write_c45``
     - added here
     - Lower-half native C45 callbacks (optional,
       ``#ifdef CONFIG_MDIO_CLAUSE_45``).
   * - ``mdio_c45_read`` / ``mdio_c45_write``
     - added here
     - C45 read/write: native preferred, falls back to reg13/14 indirect.
   * - ``CONFIG_MDIO_CLAUSE_45``
     - added here
     - C45 switch, auto-selected by ``NET_PHYLIB``.

5.2 struct phy_device_s
-----------------------

The PHY device instance, holding all runtime state:

.. code-block:: c

   struct phy_device_s
   {
     uint32_t phy_id;                        /* (PHYID1<<16)|PHYID2 */
     uint8_t  mdio_addr;                     /* MDIO address 0-31 */
     FAR struct mdio_bus_s *bus;             /* The MDIO bus it is on */
     FAR const struct phy_driver_s *drv;     /* Matched PHY driver */
     FAR struct netdev_lowerhalf_s *attached_dev;  /* Associated MAC */

     /* Link state */
     bool     is_c45;                        /* Whether it is a Clause 45 PHY */
     bool     link;                          /* Current link state */
     uint32_t speed;                         /* Current speed (10/100/1000/2500) */
     uint8_t  duplex;                        /* Current duplex mode */
     enum phy_interface_e interface;         /* Interface mode (RGMII/SGMII/...) */

     /* Capability bitmaps */
     uint32_t supported;                     /* Modes supported by the PHY */
     uint32_t advertising;                   /* Currently advertised modes */
     uint32_t lp_advertising;                /* Link-partner advertised modes */
     bool     autoneg;                       /* Auto-negotiation enable */

     /* State machine */
     enum phy_state_e state;                 /* Current state */
     struct work_s state_work;               /* LPWORK poll work */
     bool     state_machine_running;
     mutex_t  lock;                          /* Device lock, serializes state fields */

     /* Interrupt mode (optional) */
     int      irq;                           /* PHY IRQ; PHY_POLL(-1) = polling */
     bool     interrupts;                    /* Whether interrupt-driven now */

     /* Driver private data (optional) */
     FAR void *priv;                         /* PHY driver private state,
                                              * owned by the driver */
   };

Field groups:

.. list-table::
   :header-rows: 1
   :widths: 15 35 50

   * - Group
     - Key fields
     - Notes
   * - Identity
     - ``phy_id``, ``is_c45``, ``drv``
     - Who the PHY is, what protocol, which driver it binds to.
   * - Link state
     - ``link``, ``speed``, ``duplex``, ``autoneg``
     - Refreshed every second by the state machine; the MAC configures
       hardware accordingly.
   * - Capability bitmaps
     - ``supported``, ``advertising``, ``lp_advertising``
     - Core of auto-negotiation: local advertise ∩ partner advertise =
       final speed.
   * - State machine
     - ``state``, ``state_work``, ``lock``
     - The engine driving the PHY lifecycle; ``lock`` serializes state
       field access.
   * - Connections
     - ``attached_dev``, ``bus``
     - Association with the netdev and the MDIO bus.

5.3 struct phy_driver_s
-----------------------

The PHY driver callback interface:

.. code-block:: c

   struct phy_driver_s
   {
     FAR const char *name;
     uint32_t phy_id;
     uint32_t phy_id_mask;

     CODE int (*config_init)(FAR struct phy_device_s *phydev);
     CODE int (*config_aneg)(FAR struct phy_device_s *phydev);
     CODE int (*read_status)(FAR struct phy_device_s *phydev);
     CODE int (*suspend)(FAR struct phy_device_s *phydev);
     CODE int (*resume)(FAR struct phy_device_s *phydev);

     /* Interrupt support (optional; polling only if unimplemented) */
     CODE int (*config_intr)(FAR struct phy_device_s *phydev, bool enable);
     CODE int (*handle_interrupt)(FAR struct phy_device_s *phydev);
   };

.. note::

   Key design: all callbacks are optional. An unimplemented callback
   automatically falls back to the generic genphy implementation. A PHY
   compliant with IEEE 802.3 works without any vendor driver.

Match rule: ``(phydev->phy_id & drv->phy_id_mask) == drv->phy_id``

Callback execution frequency:

.. list-table::
   :header-rows: 1
   :widths: 35 20 45

   * - Callback
     - Frequency
     - Notes
   * - ``config_init``
     - once
     - Executed at connect.
   * - ``config_aneg``
     - on demand
     - When the user changes link settings.
   * - ``read_status``
     - **once per second (default)**
     - State-machine polling, the hottest path.
   * - ``suspend`` / ``resume``
     - on demand
     - On system sleep/wake.
   * - ``config_intr`` / ``handle_interrupt``
     - on demand (optional)
     - Enable INTB in interrupt mode / decode and handle all interrupt
       sources when triggered; falls back to polling if unimplemented.

6. Device Lifecycle
===================

The full PHY device lifecycle: **discover -> connect -> start -> run ->
stop -> free**.

.. note::

   NuttX MAC drivers usually already know the PHY address (from the board
   defconfig or datasheet), so they call ``phy_device_create(bus,
   known_addr)`` to create it precisely, without scanning addresses 0-31.
   ``mdio_bus_scan()`` is kept as an optional tool for the
   unknown-address case.

6.1 Discovery and Registration
------------------------------

.. code-block:: text

   MAC driver init / ifup:
     mdio_register(&priv->mdio_lower)        // register MDIO lower-half, get bus handle
     phy_device_create(handle, known_addr)   // create directly with known address
       -> read PHYID1 (reg 2) / PHYID2 (reg 3) (C45 PHY tries MMD1 indirect first)
       -> concat: phy_id = (PHYID1 << 16) | PHYID2
       -> reject invalid (0x0000/0xFFFF)
       -> allocate phy_device_s from the static pool
       -> match driver by (id & mask) -> fall back to genphy if none
       -> phydev->bus = handle (save bus handle for later access)

6.2 Connect
-----------

.. code-block:: text

   phy_connect(netdev_lower, phydev)
     -> check phydev is not in use (else -EBUSY)
     -> set bidirectional link: phydev->attached_dev = dev, dev->phydev = phydev
     -> phy_reset(phydev, 0)  // deassert, release hard reset
     -> call drv->config_init() to init the chip
     -> transition to PHY_READY

6.3 Start and Stop
------------------

.. code-block:: text

   phy_start(phydev)              // called by the MAC driver in ifup
     -> check state == PHY_READY or PHY_HALTED
     -> call drv->config_aneg()   // start auto-negotiation
     -> state = PHY_UP
     -> if irq != PHY_POLL and config_intr present: enable interrupts
     -> work_queue(LPWORK) schedules polling
          - polling mode: 1s period
          - interrupt mode: 8x period (missed-interrupt backstop only)

   phy_stop(phydev)               // called by the MAC driver in ifdown
     -> if interrupts: config_intr(false) disables interrupts
     -> work_cancel_sync(LPWORK) cancels state_work (waits for running work, prevents UAF)
     -> state = PHY_HALTED
     -> netdev_lower_carrier_off() notifies the MAC

6.4 Disconnect and Free
-----------------------

.. code-block:: text

   phy_disconnect(phydev)
     -> phy_stop() to ensure the state machine stopped
     -> clear bidirectional link (attached_dev = NULL, dev->phydev = NULL)
     -> phy_reset(phydev, 1)  // assert hard reset to finish
     -> state = PHY_DOWN

   mdio_unregister(handle)         // MAC driver frees the bus on remove

7. State Machine
================

The state machine is the core engine of PHYLIB, managing the complete PHY
lifecycle.

7.1 State Definitions
---------------------

.. code-block:: c

   enum phy_state_e
   {
     PHY_DOWN = 0,    // Device/driver not ready
     PHY_READY,       // PHY ready, MAC not started
     PHY_UP,          // Auto-negotiation started, waiting for link
     PHY_RUNNING,     // Link established
     PHY_NOLINK,      // PHY working but no link
     PHY_HALTED,      // PHY stopped
     PHY_ERROR        // Unrecoverable error (e.g. MDIO lost); forces carrier
                      // off and stops polling
   };

7.2 State Descriptions
----------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 30 30

   * - State
     - Meaning
     - Entry condition
     - Exit condition
   * - DOWN
     - Device/driver not ready
     - Initial state / after disconnect
     - phy_connect succeeds
   * - READY
     - PHY ready, MAC not open
     - config_init succeeds
     - phy_start
   * - UP
     - Auto-negotiation started, waiting for link
     - phy_start called
     - link detection result determined
   * - RUNNING
     - Link established, normal traffic
     - read_status returns link=1
     - link lost / phy_stop
   * - NOLINK
     - PHY working but no link
     - read_status returns link=0
     - link restored / phy_stop
   * - HALTED
     - Stopped, PHY suspended
     - phy_stop called
     - phy_start / disconnect
   * - ERROR
     - Unrecoverable error (MDIO lost etc.)
     - read_status / handle_interrupt returns < 0
     - phy_start (recover via stop/start or disconnect/connect)

7.3 Link Monitoring: Polling vs Interrupt
-----------------------------------------

PHYLIB supports two link-state monitoring modes, decided by the
``phy_device_s.irq`` field. Both share the same state-transition logic
(``phy_check_link``), differing only in the trigger source.

.. list-table::
   :header-rows: 1
   :widths: 12 44 44

   * - Dimension
     - Polling
     - Interrupt
   * - Enable condition
     - ``irq == PHY_POLL`` (default)
     - ``irq != PHY_POLL`` and the driver implements ``config_intr``
   * - Trigger source
     - LPWORK timer, every ``CONFIG_NET_PHYLIB_POLL_INTERVAL`` (default 1s)
     - PHY INTB pin interrupt, fired immediately on link change
   * - Response latency
     - Worst case one poll period (default 1s)
     - Near real-time (interrupt -> work schedule latency)
   * - CPU cost
     - One MDIO read per period, constant
     - Reads only on link change, zero cost when idle
   * - Backstop
     - Is the backstop itself
     - Keeps 8x low-frequency polling to catch missed interrupts

Interrupt sources — three kinds. The interrupt mode can be further
subdivided by the physical source, distinguished by the value of
``phy_device_s.irq``:

.. list-table::
   :header-rows: 1
   :widths: 12 22 26 20 20

   * - Source
     - irq value
     - Where the interrupt comes from
     - Who attaches the ISR
     - Who calls phy_mac_interrupt
   * - Pin source
     - >= 0 (real IRQ number)
     - PHY INTB pin -> SoC GPIO interrupt
     - phylib (irq_attach)
     - phylib's phy_interrupt ISR
   * - MAC source
     - PHY_MAC_INTERRUPT(-2)
     - MAC detects link change internally (e.g. SGMII in-band)
     - the MAC driver
     - the MAC driver ISR
   * - Polling
     - PHY_POLL(-1)
     - No interrupt, timed register reads
     - none
     - none (timed work backstop)

.. note::

   The phylib framework does not care about the physical source of the
   interrupt; it only exposes the unified entry ``phy_mac_interrupt()``.
   Who calls it decides the source — the pin source is called by phylib's
   own ISR, the MAC source by the MAC driver's ISR. The IRQ number is
   filled into ``phydev->irq`` by the MAC/board driver. All three sources
   converge into the same ``phy_check_link``; the link decision logic
   exists only once.

irq field three-state encoding:

.. code-block:: c

   /* phy.h: the two sentinels of the irq field */
   #define PHY_POLL          (-1)   /* no interrupt source, pure polling */
   #define PHY_MAC_INTERRUPT (-2)   /* interrupt handled by the MAC driver, phylib does not attach */
   /* others (>= 0) = a real, attachable IRQ number (pin source) */

   /* Whether it is a "real, attachable interrupt line" - i.e. a pin source */
   #define phy_interrupt_is_valid(p) \
     ((p)->irq != PHY_POLL && (p)->irq != PHY_MAC_INTERRUPT)

.. note::

   Key constraint: ``phy_interrupt`` (the pin-source ISR) runs in hard
   interrupt context. NuttX has no threaded IRQ, so it must not read MDIO
   (which would block). It therefore only "masks the line with
   up_disable_irq + posts work via phy_mac_interrupt"; the real register
   reads (handle_interrupt / read_status) are deferred to the LPWORK
   thread. This is the only structural difference from a threaded-handler
   design. The masked line is re-enabled by the worker (via up_enable_irq)
   after handle_interrupt acknowledges the source (equivalent to
   IRQF_ONESHOT).

.. note::

   In one sentence: ``phy_interrupt`` is the interrupt callback attached by
   irq_attach and called directly by hardware; ``phy_mac_interrupt`` is the
   notification API called from inside that callback (a thin forwarding
   layer) — it does not decode, read registers or touch link state, it only
   moves "something happened" safely from (possibly) interrupt context to
   the LPWORK thread, where the unified ``phy_check_link`` does the real
   work. The MAC source has no ``phy_interrupt``; the MAC driver's own ISR
   calls ``phy_mac_interrupt`` directly.

7.4 Link Detection Flow
-----------------------

Whether triggered by polling or interrupt, everything converges to the
shared ``phy_check_link()``: read the real-time PHY link state -> compare
with the current ``state`` -> on an up/down transition, notify the netdev
carrier.

7.5 Concurrency Model and Locks
-------------------------------

Who accesses the PHY / MDIO bus:

.. list-table::
   :header-rows: 1
   :widths: 40 20 40

   * - Accessor
     - Context
     - When triggered
   * - PHY state machine ``phy_check_link``
     - LPWORK thread
     - Polling: every 1s; interrupt: INTB -> work_queue
   * - MII ioctl (SIOCGMIIREG/SIOCSMIIREG)
     - Application thread
     - User space issues ioctl
   * - ethtool ioctl
     - Application thread
     - User space ``ethtool eth0`` etc.

Serialization mechanisms:

- **MDIO bus lock** (``mdio_bus_s.lock``, held by the upper-half
  ``mdio.c``): all ``mdio_read/write/c45_*`` run under this lock, ensuring
  mutual exclusion between the LPWORK state machine and application threads.
  When a MAC driver's ``mdio_ops_s`` callback is entered the upper-half
  already holds the lock, so **the callback must not take that lock again**,
  or it deadlocks.
- **PHY device lock** (``phy_device_s.lock``): protects the state fields of
  a single ``phydev`` (``state``/``link``/``speed``/``duplex``/``autoneg``/
  ``advertising``). All paths that read-modify-write state run under this
  lock, serializing state transitions and avoiding races between the state
  machine and ioctl. Initialized in ``phy_device_create`` via
  ``nxmutex_init``.
- **The state machine itself**: each ``phydev`` has only one ``state_work``,
  executed serially by a single LPWORK thread.
- **Interrupt context never touches MDIO**: ``phy_mac_interrupt()`` only
  ``work_queue()``'s a task to LPWORK (ISR-safe); the real register access
  happens in the LPWORK thread.

8. MAC Driver Integration
=========================

This section uses the real AURIX TC4x + RTL8221B driver (``aurix_enet.c``)
as a case study to detail the changes a MAC driver needs to integrate
PHYLIB.

.. note::

   Core principle: a MAC driver only needs to do two things — (1) tell
   PHYLIB how to read/write MDIO registers; (2) start/stop the state
   machine at ifup/ifdown. All PHY-chip-specific init, auto-negotiation and
   link detection are taken over by PHYLIB + the PHY driver.

8.1 Interfaces the MAC Driver Must Implement
--------------------------------------------

When integrating PHYLIB, the MAC driver connects to two groups of
interfaces: the ``netdev_lowerhalf`` OPS facing the network stack (already
present; some callbacks now forward to PHYLIB), and the MDIO lower-half
(``mdio_ops_s`` callback table, the new core part).

netdev_lowerhalf OPS (facing the stack):

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - OPS callback
     - After PHYLIB
     - Notes
   * - ``ifup``
     - modified
     - Calls ``mdio_register`` + ``phy_device_create`` + ``phy_connect`` +
       ``phy_start``.
   * - ``ifdown``
     - modified
     - Calls ``phy_stop`` + ``phy_disconnect`` + ``mdio_unregister``.
   * - ``ioctl``
     - simplified
     - MII/ethtool commands intercepted by PHYLIB; private register code
       can be removed.
   * - ``transmit`` / ``receive`` / ``txpoll``
     - unchanged
     - Pure data plane, unrelated to PHY management.

MDIO lower-half callbacks ``mdio_ops_s`` (facing the PHY, the new core):

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Callback
     - Required / optional
     - Notes
   * - ``read`` / ``write``
     - **required**
     - Clause 22 read/write; ``mdio_register`` validates ops is non-NULL.
   * - ``read_c45`` / ``write_c45``
     - optional
     - Native Clause 45; when NULL the framework uses reg13/14 indirect
       access. Implement when hardware supports native C45 (e.g. AURIX
       GETH) to save transactions.

.. note::

   Comparison: before, the MAC driver managed both the data plane and the
   PHY control plane (init / link polling / MII ioctl / MDIO), making it
   bloated and tightly coupled to the PHY chip; after, the MAC driver keeps
   only the **data plane + one group of mdio_bus callbacks**, with all PHY
   control-plane logic pushed down to PHYLIB. Swapping the PHY chip needs
   zero MAC driver change.

8.2 Change Overview
-------------------

.. list-table::
   :header-rows: 1
   :widths: 10 50 40

   * - Step
     - What
     - Where
   * - 1
     - Add mdio_lowerhalf + handle members to the struct
     - ``aurix_enet_dev_s``
   * - 2
     - Implement the MDIO read/write callbacks
     - New static functions
   * - 3
     - Register the bus + create the PHY at init
     - ``aurix_enet_ifup`` or init
   * - 4
     - connect + start in ifup
     - ``aurix_enet_ifup``
   * - 5
     - stop in ifdown
     - ``aurix_enet_ifdown``

8.3 Step 1: Add MDIO Lower-Half Members
---------------------------------------

.. code-block:: c

   struct aurix_enet_dev_s
   {
     struct netdev_lowerhalf_s         dev;
     struct aurix_enet_module_config_s config_d;
     bool                              ifup;
     bool                              mac_ready;
     const struct aurix_enet_config_s *config;
     struct work_s                     init_work;

     /* ===== New: MDIO lower-half + bus handle ===== */
     struct mdio_lowerhalf_s           mdio_lower;  /* lower-half instance (ops shell) */
     FAR struct mdio_bus_s            *mdio;        /* handle from mdio_register */

     /* ... other fields unchanged ... */
   };

.. note::

   The whole ``mdio_bus_s`` bus object is no longer embedded; only the
   lower-half ``mdio_lowerhalf_s`` (a thin ops vtable shell) and the bus
   handle pointer returned by the upper-half are kept. The bus core object
   (including the lock) is allocated and held by the upper-half
   ``drivers/net/mdio.c``. In multi-instance cases (BMP multi-core /
   multi-port) each ``aurix_enet_dev_s`` has its own lower, and the callback
   uses ``container_of`` to recover the instance.

8.4 Step 2: Implement the MDIO Lower-Half Callbacks
---------------------------------------------------

Wrap the hardware MDIO controller into ``mdio_ops_s`` callbacks. Note the
community signature: ``lower`` is the lower-half instance, the read value
is returned via the ``value`` out-parameter, and the return is 0 / negative
errno.

.. code-block:: c

   /* Clause 22 read - called via ops->read */
   static int aurix_mdio_c22_read(FAR struct mdio_lowerhalf_s *lower,
                                  uint8_t phyaddr, uint8_t regaddr,
                                  FAR uint16_t *value)
   {
     FAR struct aurix_enet_dev_s *priv =
       container_of(lower, struct aurix_enet_dev_s, mdio_lower);

     if (IfxGeth_Eth_mdio_read_clause22(priv->config_d.geth,
                                        priv->config->port,
                                        phyaddr, regaddr, value))
       {
         return -EIO;
       }

     return OK;
   }

   /* Clause 22 write */
   static int aurix_mdio_c22_write(FAR struct mdio_lowerhalf_s *lower,
                                   uint8_t phyaddr, uint8_t regaddr,
                                   uint16_t val)
   {
     FAR struct aurix_enet_dev_s *priv =
       container_of(lower, struct aurix_enet_dev_s, mdio_lower);

     if (IfxGeth_Eth_mdio_write_clause22(priv->config_d.geth,
                                         priv->config->port,
                                         phyaddr, regaddr, val))
       {
         return -EIO;
       }

     return OK;
   }

   /* Native Clause 45 read/write (AURIX GETH supports it, avoids reg13/14) */
   static int aurix_mdio_c45_read(FAR struct mdio_lowerhalf_s *lower,
                                  uint8_t phyaddr, uint8_t devad,
                                  uint16_t regaddr, FAR uint16_t *value)
   {
     FAR struct aurix_enet_dev_s *priv =
       container_of(lower, struct aurix_enet_dev_s, mdio_lower);

     if (IfxGeth_Eth_mdio_read_clause45(priv->config_d.geth,
                                        priv->config->port,
                                        phyaddr, devad, regaddr, value))
       {
         return -EIO;
       }

     return OK;
   }

   static int aurix_mdio_c45_write(FAR struct mdio_lowerhalf_s *lower,
                                   uint8_t phyaddr, uint8_t devad,
                                   uint16_t regaddr, uint16_t val)
   {
     FAR struct aurix_enet_dev_s *priv =
       container_of(lower, struct aurix_enet_dev_s, mdio_lower);

     if (IfxGeth_Eth_mdio_write_clause45(priv->config_d.geth,
                                         priv->config->port,
                                         phyaddr, devad, regaddr, val))
       {
         return -EIO;
       }

     return OK;
   }

PHY hardware reset callback ``aurix_mdio_reset`` (level control):

.. code-block:: c

   /* PHY hard reset line (PHYRSTB) control. value!=0 asserts (drive low,
    * into reset); value==0 deasserts (drive high, release). The reset pin
    * is selected by GETH port (port0->P10, port1->P11). The lower half
    * performs the requested level flip + the datasheet assert/settle
    * delays; any idempotency is the lower half's responsibility.
    */
   static int aurix_mdio_reset(FAR struct mdio_lowerhalf_s *lower,
                               uint8_t phyaddr, int value)
   {
     FAR struct aurix_enet_dev_s *priv =
       container_of(lower, struct aurix_enet_dev_s, mdio_lower);
     int pin = (priv->config->port == 0) ? 10 : 11;

     IfxPort_setPinModeOutput(priv->config_d.port_p, pin,
                              IfxPort_OutputMode_pushPull,
                              IfxPort_OutputIdx_general);
     if (value)
       {
         IfxPort_setPinLow(priv->config_d.port_p, pin);   /* assert */
         up_mdelay(10);                                   /* Tgap >= 10ms */
       }
     else
       {
         IfxPort_setPinHigh(priv->config_d.port_p, pin);  /* deassert */
         up_mdelay(68);                                   /* settle 68ms */
       }

     return OK;
   }

.. note::

   PHYLIB also provides a thin inline wrapper ``phy_reset(phydev, value)``
   (forwarding to ``mdio_reset(phydev->bus, phydev->mdio_addr, value)``),
   called by ``phy_connect`` (deassert) / ``phy_disconnect`` (assert)
   during the lifecycle.

8.5 Step 3: Register MDIO + Create PHY at ifup
----------------------------------------------

.. note::

   MDIO callback execution-context constraint: ``read``/``write``/
   ``read_c45``/``write_c45`` are triggered by two kinds of callers — (1)
   the PHY state machine's ``read_status`` (running in the **LPWORK
   thread**); (2) ethtool/MII ioctl (running in an **application thread**).
   Both are serialized by the upper-half bus lock. Therefore the callback
   implementation may sleep/block (it is always in thread context, never
   in interrupt/atomic context), and must not re-acquire the upper-half bus
   lock (the framework already holds it; doing so deadlocks).

.. code-block:: c

   /* In aurix_enet_ifup(), after aurix_mdio_init() */

   static int aurix_phylib_setup(struct aurix_enet_dev_s *priv)
   {
     /* MDIO lower-half ops table (file-scope static const) */
     static const struct mdio_ops_s g_aurix_mdio_ops =
     {
       .read      = aurix_mdio_c22_read,
       .write     = aurix_mdio_c22_write,
   #ifdef CONFIG_MDIO_CLAUSE_45
       .read_c45  = aurix_mdio_c45_read,
       .write_c45 = aurix_mdio_c45_write,
   #endif
       .reset     = aurix_mdio_reset,   /* PHY hard reset line (PHYRSTB) level control */
     };

     int ret;

     /* 1. Register the vendor PHY driver once at boot (not idempotent:
      * each call consumes a driver-table slot, so do not call repeatedly).
      */
     rtl8221b_register();

     /* 2. Fill the lower-half and register with the upper-half, get the handle */
     priv->mdio_lower.ops = &g_aurix_mdio_ops;

     priv->mdio = mdio_register(&priv->mdio_lower);
     if (priv->mdio == NULL)
       {
         nerr("ERROR: mdio_register failed\n");
         return -ENODEV;
       }

     /* 3. Hard-reset the PHY (must complete before reading the PHY ID).
      * assert->deassert forms a complete power-on pulse; the lower-half
      * applies 10ms assert + 68ms settle delays.
      */
     mdio_reset(priv->mdio, ETH_PHY_ADDRESS, 1);   /* assert  */
     mdio_reset(priv->mdio, ETH_PHY_ADDRESS, 0);   /* deassert + settle */

     /* 4. Create the PHY device (known address, read PHY ID, match RTL8221B) */
     priv->phydev = phy_device_create(priv->mdio, ETH_PHY_ADDRESS);
     if (priv->phydev == NULL)
       {
         nerr("ERROR: phy_device_create failed\n");
         mdio_unregister(priv->mdio);
         priv->mdio = NULL;
         return -ENODEV;
       }

     /* 5. Connect the PHY to the netdev (runs config_init / firmware patch) */
     ret = phy_connect(&priv->dev, priv->phydev);
     if (ret < 0)
       {
         nerr("ERROR: phy_connect failed: %d\n", ret);
         mdio_unregister(priv->mdio);
         priv->mdio = NULL;
         priv->phydev = NULL;
         return ret;
       }

     return OK;
   }

8.6 Step 4: Start the State Machine in ifup
-------------------------------------------

.. code-block:: c

   static int aurix_enet_ifup(struct netdev_lowerhalf_s *dev)
   {
     struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;

     /* ... hardware init (hsphy_init, geth_init, mdio_init) ... */

   #ifdef CONFIG_NET_PHYLIB
     /* Integrate PHYLIB: register bus + create PHY + connect */
     ret = aurix_phylib_setup(priv);
     if (ret < 0)
       {
         return ret;
       }

     /* Start the PHY state machine - begin auto-negotiation + link polling */
     phy_start(priv->dev.phydev);

   #else
     /* Old path: call MCAL PHY init directly (kept for compatibility) */
     ret = aurix_realtekethphy_init(priv);
   #endif

     /* ... remaining init (DMA, interrupts, MAC enable) ... */
   }

.. note::

   After ``phy_start()`` the link state is managed automatically by the
   PHYLIB state machine. When the PHY detects link up, PHYLIB calls
   ``netdev_lower_carrier_on()`` and the stack starts sending/receiving.
   The MAC driver no longer judges link state manually.

8.7 Step 5: Stop the State Machine in ifdown
--------------------------------------------

.. code-block:: c

   static int aurix_enet_ifdown(struct netdev_lowerhalf_s *dev)
   {
     struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;

   #ifdef CONFIG_NET_PHYLIB
     /* Stop the PHY state machine */
     if (priv->dev.phydev != NULL)
       {
         phy_stop(priv->dev.phydev);
       }
   #endif

     /* ... remaining cleanup (disable interrupts, stop MAC) ... */
   }

8.8 Benefits After Integration
------------------------------

#. Removes about 200 lines of PHY management code
   (``aurix_realtekethphy_init`` + ``aurix_ethlink_up`` + link-retry logic).
#. Gets ethtool support automatically (speed/duplex/autoneg query and set).
#. Link changes are reported to the stack automatically, no manual
   carrier_on/off.
#. Swapping the PHY chip only needs a vendor driver file; zero MAC change.
#. Unified MII ioctl behavior, no longer implemented per driver.

9. Master/Slave Core Link Sync in BMP Multi-core
================================================

9.1 Background: the Shared MAC Model under BMP
----------------------------------------------

AURIX TC4x in BMP (Bare Metal Porting / AMP) mode runs an independent
NuttX instance on each core (``CONFIG_NCPUS=6``, ``CONFIG_SMP_NCPUS=1`` per
core), communicating through shared memory. They share the same GETH MAC
hardware, but each core has its own ``aurix_enet_dev_s`` / netdev.

.. note::

   The MDIO controller and the PHY are a globally unique physical resource.
   If 6 independent OS instances accessed MDIO / ran the PHY state machine
   simultaneously, it would cause bus contention and state corruption.
   Therefore a single owner must be established.

Following the "control-plane / data-plane separation" principle of
SR-IOV (the PF manages the PHY, the VFs only send/receive and query the PF
for link), the BMP design is:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Role
     - Maps to
     - Responsibility
   * - main_core
     - SR-IOV PF
     - Owns MDIO / PHY exclusively, runs the phylib state machine,
       **publishes** link state.
   * - non-main_core
     - SR-IOV VF
     - Does not touch the PHY, **subscribes** to link state, reflects it to
       its own netdev carrier.

The difference: SR-IOV uses a hardware mailbox; BMP uses **shared memory**
(``shared_data_manual``) to pass link state.

Before (no phylib): there was no real master/slave link sync. main_core
polled the PHY UTP link via the iLLD ``aurix_ethlink_up()`` (with retry)
and did ``carrier_on``; non-main_core did an **unconditional**
``netdev_lower_carrier_on()``, i.e. assumed carrier was always up and never
knew the real physical link.

After (phylib + shared memory): main_core's phylib state machine manages
the link uniformly and publishes it through the shared-memory flag
``eth_link_up``; non-main_core subscribes to that flag and **really
follows** the physical link up/down.

9.2 Shared Memory Field
-----------------------

Add a link flag to ``shared_data_manual_s`` in
``vendor/infineon/chips/aurix/include/chip.h``:

.. code-block:: c

   typedef struct
   {
     volatile int        tmadc_sync_barrier;
     volatile int        eth_sync_barrier;
     volatile bool       eth_mac_ready_flag;

     /* ENET PHY link state published by the main core (which owns the PHY
      * and runs the phylib state machine) so that non-main cores can follow
      * the real link status instead of assuming carrier is always up.
      */
     volatile bool       eth_link_up;
     struct can_module_s can_module[CAN_MODULE_NUM];
     volatile int        leth_sync_barrier;
   } shared_data_manual_s;

.. note::

   ``eth_link_up`` is marked ``volatile``, consistent with the existing
   ``eth_sync_barrier``. After main_core writes it, it calls ``UP_DMB()``
   for cross-core visibility. The shared region lives in the RPTUN shared
   segment, visible to all cores.

9.3 Writer: main_core Publishes Link State
-------------------------------------------

main_core owns the PHY; the phylib state machine manages its own carrier.
It additionally runs a periodic ``aurix_enet_link_worker`` mirroring
``phydev->link`` to shared memory:

.. code-block:: c

   static void aurix_enet_link_worker(void *arg)
   {
     struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)arg;

     /* ifdown cleared priv->ifup, so the worker stops naturally */
     if (!priv->ifup)
       {
         return;
       }

     if (priv->config->main_core)
       {
         if (priv->phydev != NULL)
           {
             /* Publish the current link state for other cores to read */
             shared_data_manual.eth_link_up = priv->phydev->link;
             UP_DMB();
           }
       }
     else
       {
         /* non-main_core: see below */
       }

     work_queue(LPWORK, &priv->link_work, aurix_enet_link_worker,
                priv, MSEC2TICK(CONFIG_NET_PHYLIB_POLL_INTERVAL));
   }

9.4 Reader: non-main_core Follows Link State
--------------------------------------------

non-main_core has no phydev and runs no state machine; in the same
``link_worker`` it reads the shared flag and, using edge detection
(``link_up_last``), updates its own carrier only on change:

.. code-block:: c

     else
       {
         /* non-main_core: follow the link state published by main_core */
         bool link = shared_data_manual.eth_link_up;

         if (link != priv->link_up_last)
           {
             priv->link_up_last = link;
             if (link)
               {
                 netdev_lower_carrier_on(&priv->dev);
               }
             else
               {
                 netdev_lower_carrier_off(&priv->dev);
               }
           }
       }

non-main_core does ``netdev_lower_carrier_off()`` first at ifup (initial
``link_up_last = false``), guaranteeing it starts from carrier-off
regardless of the current link, then the worker raises it per the shared
flag.

.. note::

   This mechanism is only enabled under ``CONFIG_AURIX_ENET_USE_PHY &&
   CONFIG_NET_PHYLIB``. Single-core or non-PHYLIB cases fall back to the
   original behavior (direct carrier_on).

10. User-space PHY Control Call Chains
======================================

User space controls the PHY through ioctl system calls, in two kinds by
cmd:

- **SIOCETHTOOL**: link parameter query/set (used by the ethtool tool).
- **SIOCGMIIREG / SIOCSMIIREG**: direct PHY register read/write.

10.1 Path 1: ethtool ioctl (link parameters)
---------------------------------------------

.. code-block:: text

   User: ethtool eth0 / ethtool -s eth0 speed 100
       | ioctl(fd, SIOCETHTOOL, &ifr)
   Socket layer: net/netdev/netdev_ioctl.c -> netdev_ioctl()
       | find net_device by ifr_name, cmd == SIOCETHTOOL
       | call dev->d_ioctl(dev, cmd, arg)
   Upper half: drivers/net/netdev_upperhalf.c -> netdev_upper_ioctl()
       | cmd == SIOCETHTOOL && lower->phydev != NULL
   PHYLIB: phy_upper_ethtool_ioctl(lower->phydev, arg)
       | switch(ethtool_cmd)
       |-- ETHTOOL_GLINKSETTINGS -> phy_ethtool_get_link_ksettings()
       |-- ETHTOOL_SLINKSETTINGS -> phy_ethtool_set_link_ksettings()
       |-- ETHTOOL_NWAY_RST      -> phy_ethtool_nway_reset()
       |-- ETHTOOL_GLINK         -> phydev->link
       | (if PHYLIB returns -ENOTTY, fall through to the MAC's own ioctl)
   Upper half: lower->ops->ioctl(dev, cmd, arg)

10.2 Path 2: MII ioctl (direct register access)
------------------------------------------------

.. code-block:: text

   User: ioctl(fd, SIOCGMIIREG, &ifr)
       | struct mii_ioctl_data_s passes phy_id and reg_num
   Socket layer: net/netdev/netdev_ioctl.c -> netdev_ioctl()
       | cmd == SIOCGMIIREG/SIOCSMIIREG/SIOCGMIIPHY
       | call dev->d_ioctl(dev, cmd, arg)
   Upper half: drivers/net/netdev_upperhalf.c -> netdev_upper_ioctl()
       | lower->phydev != NULL
   PHYLIB: phy_upper_mii_ioctl(lower->phydev, cmd, arg)
       |-- SIOCGMIIPHY -> return phydev->mdio_addr
       |-- SIOCGMIIREG -> phy_read(phydev, reg_num)
       |-- SIOCSMIIREG -> phy_write(phydev, reg_num, val)
       |
   phy_core.c: phy_read() / phy_write()
       |
   mdio.c (upper half): mdio_read/mdio_write - locked, calls lower->ops->read/write
       |
   MAC hardware MDIO controller -> PHY chip registers

10.3 Command to Kernel Call Mapping
-----------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - ethtool command
     - Kernel CMD
     - Final call
   * - ``ethtool eth0``
     - ETHTOOL_GLINKSETTINGS
     - ``phy_ethtool_get_link_ksettings()``
   * - ``ethtool -s eth0 speed 100``
     - ETHTOOL_SLINKSETTINGS
     - ``phy_ethtool_set_link_ksettings()`` -> ``drv->config_aneg()``
   * - MII register read
     - SIOCGMIIREG
     - ``phy_upper_mii_ioctl()`` -> ``phy_read()`` -> ``mdio_read()``
   * - MII register write
     - SIOCSMIIREG
     - ``phy_upper_mii_ioctl()`` -> ``phy_write()`` -> ``mdio_write()``

.. note::

   Key design: ``netdev_upper_ioctl`` first checks ``lower->phydev !=
   NULL`` and gives the command to PHYLIB first. Only when PHYLIB returns
   ``-ENOTTY`` (does not handle this command) does it fall through to the
   MAC driver's own ``lower->ops->ioctl``. This guarantees unified behavior
   with PHYLIB and backward compatibility without it.

11. Debugging and Verification
==============================

11.1 ethtool Commands
---------------------

With ``CONFIG_SYSTEM_ETHTOOL=y``, in NSH:

.. code-block:: text

   core1> ethtool eth0
   Settings for eth0:
       Supported link modes:
           10baseT/Full 100baseT/Full 1000baseT/Full Autoneg 2500baseT/Full
       Supports auto-negotiation: Yes
       Speed: 1000Mb/s
       Duplex: Full
       Auto-negotiation: on
       PHY Address: 2
       Link detected: yes

Supported commands:

- ``ethtool eth0`` — show link settings and status
- ``ethtool -s eth0 speed 100 duplex full`` — change link parameters
- ``ethtool -r eth0`` — restart auto-negotiation

11.2 Kernel Log
---------------

PHYLIB emits key events via ``ninfo()``/``nerr()`` (needs
``CONFIG_DEBUG_NET_INFO=y``):

.. code-block:: text

   [CPU1] phylib: PHY id=0x001cc849 addr=2 drv='RTL8221B 2.5Gbps PHY' c45=1
   [CPU1] phylib: phy_start config_aneg ret=0
   [CPU1] phylib: PHY link up: 1000 Mbps full
   [CPU1] phylib: PHY link down

12. Performance Overhead Evaluation
===================================

To quantify the extra overhead the PHYLIB abstraction adds to the PHY
register access path, the most common path — MII register read
(``SIOCGMIIREG``) — was measured.

Method: a start timestamp is recorded at the entry of the user-space
``mdio`` tool and an end timestamp on the kernel-side path; the difference
is the software-path time of that access. Timestamps use
``perf_gettime() * 1000000 / perf_getfreq()`` in microseconds. Each path is
sampled 100 times and averaged.

.. list-table::
   :header-rows: 1
   :widths: 15 70 15

   * - Path
     - Call chain
     - Avg time
   * - No PHYLIB
     - ``ioctl`` -> netdev_upper_ioctl -> ``lower->ops->ioctl`` ->
       aurix_enet_ioctl -> private MDIO read
     - ~13 us
   * - With PHYLIB
     - ``ioctl`` -> netdev_upper_ioctl -> phy_upper_mii_ioctl -> phy_read
       -> mdio_bus -> MDIO callback
     - ~12 us

.. note::

   Conclusion: the PHYLIB abstraction adds almost no overhead to the MII
   register access path. The two paths average essentially the same (13 us
   vs 12 us), a difference on the order of 1 us. The bulk of the 13 us is
   the user-to-kernel syscall cost of ioctl; the PHYLIB forwarding chain
   itself is only a small part. Introducing PHYLIB does not observably
   affect PHY register access performance.

13. Framework Extensions (Implemented)
=======================================

On top of the base framework, the following capabilities were added to
support paged PHYs and automotive single-pair Ethernet (100BASE-T1) PHYs.
All are backward compatible (removed at compile time when
``CONFIG_NET_PHYLIB=n``).

13.1 Driver Private Data phy_device_s.priv
------------------------------------------

``struct phy_device_s`` gains a ``FAR void *priv`` pointer for a PHY driver
to keep chip-specific state across callbacks (config_init / read_status /
suspend / resume), such as DVDDIO voltage selection, operation state or
patch-load flags — fields beyond IEEE 802.3 that should not go into the
generic structure.

- **Ownership**: ``phydev->priv`` is owned entirely by the PHY driver. The
  framework only zero-initializes it in ``phy_device_create()`` and never
  dereferences or frees it. A driver that stores heap memory here is
  responsible for releasing it in its own teardown; a driver may also use
  it as a plain per-device flag without allocating.

13.2 100BASE-T1 / 1000BASE-T1 Capability Bits
---------------------------------------------

The link capability bitmap gains two automotive single-pair Ethernet modes:

.. code-block:: c

   #define SUPPORTED_100BASET1_FULL  (1 << 13)
   #define SUPPORTED_1000BASET1_FULL (1 << 14)

The PHY driver fills ``phydev->supported`` accordingly in config_init, and
it is passed to user space through ethtool.

13.3 Paged Register Access
--------------------------

Some PHYs (e.g. Realtek) keep most registers on a non-default page;
accessing them needs "switch page (reg 31) -> read/write target -> restore
page", done atomically so MII ioctl cannot change the page register in
between. Three new APIs:

.. code-block:: c

   int phy_read_paged(phydev, page, regaddr, value);
   int phy_write_paged(phydev, page, regaddr, value);
   int phy_modify_paged(phydev, page, regaddr, mask, set);

Implementation points:

- Built from internal helpers ``phy_select_page()`` (lock + save old page +
  switch) and ``phy_restore_page()`` (restore old page + unlock + propagate
  error); the three APIs just call this pair.
- Hold the **MDIO bus lock** throughout, mutually exclusive with MII ioctl;
  switch only when the page differs (dedup).

13.4 MDIO Bus Lock Primitives
-----------------------------

To support the "hold the lock across multiple transactions" need of paged
access, the MDIO upper-half adds:

.. code-block:: c

   int  mdio_lock(dev);     void mdio_unlock(dev);          /* explicit lock/unlock */
   int  mdio_read_nolock(dev, phyaddr, regaddr, value);     /* lockless read when held */
   int  mdio_write_nolock(dev, phyaddr, regaddr, value);    /* lockless write when held */

A caller composes an atomic multi-transaction sequence (such as paged
access) between ``mdio_lock()`` / ``mdio_unlock()`` using the ``*_nolock``
primitives; the lock encapsulation stays in the MDIO layer and phylib never
touches the bus lock object directly.

14. Extensibility for PHY-side Features (PTP, WoL, EEE, ...)
============================================================

Some PHYs integrate features that traditionally live on the MAC side, such
as IEEE 1588 / PTP hardware timestamping (e.g. TI DP83630) and
Wake-on-LAN. PHYLIB supports such features as a framework-level
extensibility model rather than hard-coding each one into the core, so
every addition is opt-in and PHYs without the feature are unaffected.

14.1 The Four Extension Points
------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Extension point
     - Role
   * - Optional ``phy_driver_s`` callbacks
     - The core (state machine, matching, lifecycle) only knows the callback
       table. A new feature adds a group of *optional* callbacks, exactly
       like ``config_intr`` / ``handle_interrupt`` / ``read_page`` today.
       Unimplemented callbacks fall back to genphy; appending callbacks at
       the end of the struct keeps the ABI stable for existing drivers.
   * - ``phy_ioctl()`` dispatch
     - All MII/ethtool commands are offered to ``phy_ioctl()`` first by the
       netdev upper-half, falling back to the MAC only on ``-ENOTTY``. A new
       feature adds its ethtool command branch here, reached from user space
       through the standard ethtool interface.
   * - ``phy_read_mmd`` / ``phy_*_paged`` + interrupt path
     - Feature registers usually live in MMD or vendor pages, reached with
       the existing accessors — no new mechanism needed. Asynchronous events
       (timestamp ready, wake) reuse ``config_intr`` / ``handle_interrupt``.
   * - ``phydev->priv``
     - Per-chip state outside IEEE 802.3 (PTP clock context, WoL match
       rules, ...) lives here without polluting the generic
       ``phy_device_s``.

Adding a PHY-side feature therefore means: define the optional callbacks,
dispatch the matching ethtool command in ``phy_ioctl()``, reach the
registers via MMD/paged accessors, deliver events through the interrupt
path, and keep private state in ``phydev->priv``. The framework core does
not change.

14.2 Worked Example: Wake-on-LAN
--------------------------------

WoL is a PHY-side-only capability and closes the loop entirely with the
extension points above:

- Add optional ``get_wol`` / ``set_wol`` callbacks to the vendor driver,
  mapped to the ethtool WoL commands. (Those ethtool commands would be added
  to the NuttX ethtool layer when first needed; NuttX does not define them
  yet.)
- Store the wake / magic-packet match rules in ``phydev->priv``.
- Deliver the wake event through ``handle_interrupt``.

14.3 Worked Example: PTP / Timestamping
---------------------------------------

Capability discovery is reported from a PHY-side callback that fills
``struct ethtool_ts_info`` and is returned via ``ETHTOOL_GET_TS_INFO``
(PHC index and the supported ``SOF_TIMESTAMPING_*`` modes).

Full PTP, however, is more than a PHY callback. Per-packet hardware
timestamping also requires the networking data plane:

- Per-packet ``SO_TIMESTAMPING`` delivery to user space (currently
  incomplete in NuttX — the flags are accepted but not yet honoured
  individually).
- A ``SIOCSHWTSTAMP``-style configuration interface, which NuttX does not
  provide yet.

So PHYLIB contributes the capability-discovery callback and the PHY 1588
register configuration (over MMD), while the data-plane timestamp path is a
larger, separately tracked work item outside the PHY abstraction.

.. note::

   Scope: these callbacks (WoL ``get_wol``/``set_wol``, PTP ``ts_info`` and
   timestamp config) are the reserved extension path and are not implemented
   in the current series, which provides the base framework, link management
   and the Realtek PHY drivers. Adding them requires no change to existing
   PHY drivers.

15. Future Plans
================

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Feature
     - Priority
     - Notes
   * - Fixup mechanism
     - high
     - Board-level code to correct PHY behavior without changing driver
       source.
   * - PHY Package
     - medium
     - Multi-port PHY chips sharing global configuration.
   * - ethtool
     - high
     - More ethtool command support.
   * - PTP / WoL / EEE
     - medium
     - PHY-side extended features via the extension points in Section 14.
