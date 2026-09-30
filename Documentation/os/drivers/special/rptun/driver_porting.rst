=========================
Driver Porting Guide
=========================

The RPTUN framework is designed to be portable. It decouples from specific
hardware platforms through a set of callback functions defined in the
``rptun_ops`` structure. To support RPTUN on a new hardware platform,
platform driver developers must implement the following core interfaces
to meet the framework's operational requirements.

Driver Core Responsibilities
============================

Provide Resource Information
----------------------------

- The driver must define shared memory regions and correctly populate the
  Resource Table within them.

- The driver needs to provide the Resource Table or remote firmware address
  to the RPTUN framework through the ``get_resource()`` or ``get_firmware()``
  interface.

Implement Core Operation Interfaces (rptun_ops)
-----------------------------------------------

**Lifecycle Management**

Provide underlying implementations for starting (``start``) and stopping
(``stop``) the remote core.

**Inter-core Communication**

- ``notify``: Implement the logic for sending notifications to the remote
  core (typically triggering a hardware interrupt).

- ``register_callback``: Register a callback function that should be called
  by the underlying interrupt service routine when receiving interrupts
  from the remote core.

**Address Space Translation**

If the address spaces of the host core and remote core are inconsistent,
the ``get_addrenv()`` interface must be implemented to provide the address
translation table.

Associate Hardware Interrupts
-----------------------------

In the platform's Interrupt Service Routine (ISR), it must be able to
identify hardware interrupts from the remote core and accurately call
the callback function registered through ``register_callback``.

rptun_ops Structure
===================

Each RPTUN driver must implement an instance of ``struct rptun_ops_s``:

.. code-block:: c

   struct rptun_ops_s
   {
     CODE FAR const char *(*get_local_cpuname)(FAR struct rptun_dev_s *dev);
     CODE FAR const char *(*get_cpuname)(FAR struct rptun_dev_s *dev);
     CODE FAR const char *(*get_firmware)(FAR struct rptun_dev_s *dev);

     CODE FAR const struct rptun_addrenv_s *(*get_addrenv)(
                           FAR struct rptun_dev_s *dev);
     CODE FAR struct rptun_rsc_s *(*get_resource)(FAR struct rptun_dev_s *dev);

     CODE bool (*is_autostart)(FAR struct rptun_dev_s *dev);
     CODE bool (*is_master)(FAR struct rptun_dev_s *dev);

     CODE int (*config)(struct rptun_dev_s *dev, void *data);
     CODE int (*start)(FAR struct rptun_dev_s *dev);
     CODE int (*stop)(FAR struct rptun_dev_s *dev);
     CODE int (*notify)(FAR struct rptun_dev_s *dev, uint32_t vqid);
     CODE int (*register_callback)(FAR struct rptun_dev_s *dev,
                                   rptun_callback_t callback, FAR void *arg);

     CODE void (*reset)(FAR struct rptun_dev_s *dev, int value);
     CODE void (*panic)(FAR struct rptun_dev_s *dev);
   };

Interface Descriptions
======================

get_local_cpuname
-----------------

Returns the local CPU name string.

get_cpuname
-----------

Returns the remote CPU name string. This name is used to identify the
communication channel and register the character device.

get_firmware
------------

Returns the remote firmware file path. If the remote core firmware needs
to be loaded by the host core, this interface should return the firmware
path.

get_addrenv
-----------

Returns the address environment translation table. Used when physical
addresses differ between cores.

.. code-block:: c

   struct rptun_addrenv_s
   {
     uintptr_t pa;    /* Physical address on local core */
     uintptr_t da;    /* Device address on remote core */
     size_t    size;  /* Size of the memory region */
   };

get_resource
------------

Returns a pointer to the Resource Table. The driver must prepare the
Resource Table in shared memory and return its address through this
interface.

is_autostart
------------

Returns whether to automatically start the remote core. If ``true``,
``rptun_initialize()`` will automatically call ``rptun_dev_start()``.

is_master
---------

Returns whether the local core is the master. The master core is typically
responsible for initializing shared memory and the Resource Table.

config
------

Configures the remote core. Called before starting the remote core.

start
-----

Starts the remote core. Implements the hardware-specific startup sequence.

stop
----

Stops the remote core. Implements the hardware-specific shutdown sequence.

notify
------

Sends a notification to the remote core. Typically implemented by triggering
a hardware interrupt (e.g., mailbox, software interrupt).

register_callback
-----------------

Registers a callback function for receiving notifications from the remote
core. The driver's ISR should call this callback when an inter-core
interrupt is received.

reset
-----

Resets the remote core with the specified reset value.

panic
-----

Triggers a panic state on the remote core.

Example Implementation
======================

Here's a minimal example of implementing RPTUN driver operations:

.. code-block:: c

   static FAR const char *myboard_rptun_get_cpuname(FAR struct rptun_dev_s *dev)
   {
     return "remote";
   }

   static FAR struct rptun_rsc_s *
   myboard_rptun_get_resource(FAR struct rptun_dev_s *dev)
   {
     return &g_rptun_rsc;  /* Pre-defined resource table */
   }

   static bool myboard_rptun_is_master(FAR struct rptun_dev_s *dev)
   {
     return true;
   }

   static int myboard_rptun_start(FAR struct rptun_dev_s *dev)
   {
     /* Hardware-specific remote core startup */
     return OK;
   }

   static int myboard_rptun_stop(FAR struct rptun_dev_s *dev)
   {
     /* Hardware-specific remote core shutdown */
     return OK;
   }

   static int myboard_rptun_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
   {
     /* Trigger inter-core interrupt */
     return OK;
   }

   static int myboard_rptun_register_callback(FAR struct rptun_dev_s *dev,
                                              rptun_callback_t callback,
                                              FAR void *arg)
   {
     /* Store callback for ISR to use */
     g_callback = callback;
     g_arg = arg;
     return OK;
   }

   static const struct rptun_ops_s g_myboard_rptun_ops =
   {
     .get_cpuname       = myboard_rptun_get_cpuname,
     .get_resource      = myboard_rptun_get_resource,
     .is_master         = myboard_rptun_is_master,
     .start             = myboard_rptun_start,
     .stop              = myboard_rptun_stop,
     .notify            = myboard_rptun_notify,
     .register_callback = myboard_rptun_register_callback,
   };

Existing Implementations
========================

Reference implementations can be found in the following files:

- ``arch/sim/src/sim/sim_rptun.c`` - Simulator RPTUN driver
- ``arch/arm/src/stm32h7/stm32_rptun.c`` - STM32H7 RPTUN driver
- ``arch/arm/src/imx9/imx9_rptun.c`` - i.MX9 RPTUN driver
- ``arch/arm/src/nrf53/nrf53_rptun.c`` - nRF53 RPTUN driver
- ``arch/risc-v/src/k230/k230_rptun.c`` - K230 RPTUN driver
- ``arch/risc-v/src/qemu-rv/qemu_rv_rptun.c`` - QEMU RISC-V RPTUN driver
