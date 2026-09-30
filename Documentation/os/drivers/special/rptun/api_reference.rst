=============
API Reference
=============

This section describes the kernel APIs and NSH commands available for
RPTUN operations.

Kernel API
==========

rptun_initialize
----------------

.. c:function:: int rptun_initialize(FAR struct rptun_dev_s *dev)

   Initialize and register an RPTUN device.

   :param dev: Pointer to the RPTUN device structure with ops configured
   :return: OK on success; a negated errno value on failure

   This function creates an RPTUN instance and registers it with the system.
   If the device is configured for auto-start, it will also start the
   remote core.

rptun_boot
----------

.. c:function:: int rptun_boot(FAR const char *cpuname)

   Start the remote CPU.

   :param cpuname: Name of the remote CPU to start
   :return: OK on success; a negated errno value on failure

   Example:

   .. code-block:: c

      /* Start remote CPU named "M33" */
      ret = rptun_boot("M33");
      if (ret < 0)
        {
          syslog(LOG_ERR, "Failed to boot remote CPU: %d\n", ret);
        }

rptun_poweroff
--------------

.. c:function:: int rptun_poweroff(FAR const char *cpuname)

   Stop the remote CPU.

   :param cpuname: Name of the remote CPU to stop
   :return: OK on success; a negated errno value on failure

   Example:

   .. code-block:: c

      /* Stop remote CPU named "M33" */
      ret = rptun_poweroff("M33");

rptun_reset
-----------

.. c:function:: int rptun_reset(FAR const char *cpuname, int value)

   Reset the remote CPU.

   :param cpuname: Name of the remote CPU to reset
   :param value: Reset value (implementation-specific)
   :return: OK on success; a negated errno value on failure

   Example:

   .. code-block:: c

      /* Reset remote CPU named "M33" */
      ret = rptun_reset("M33", 0);

IOCTL Commands
==============

The RPTUN character device supports the following ioctl commands:

RPTUNIOC_START
--------------

Start the remote CPU associated with this RPTUN device.

.. code-block:: c

   int fd = open("/dev/rptun/remote", O_RDWR);
   ret = ioctl(fd, RPTUNIOC_START, 0);

RPTUNIOC_STOP
-------------

Stop the remote CPU associated with this RPTUN device.

.. code-block:: c

   ret = ioctl(fd, RPTUNIOC_STOP, 0);

RPTUNIOC_RESET
--------------

Reset the remote CPU associated with this RPTUN device.

.. code-block:: c

   int reset_value = 0;
   ret = ioctl(fd, RPTUNIOC_RESET, reset_value);

NSH Commands
============

The ``rptun`` NSH command provides user-space control over RPTUN devices.

rptun start
-----------

Start the remote core.

::

   nsh> rptun start /dev/rptun/<cpuname>

Example:

::

   nsh> rptun start /dev/rptun/M33

rptun stop
----------

Stop the remote core.

::

   nsh> rptun stop /dev/rptun/<cpuname>

Example:

::

   nsh> rptun stop /dev/rptun/M33

Configuration Options
=====================

The following Kconfig options are available for RPTUN:

CONFIG_RPTUN
------------

Enable RPTUN driver support.

CONFIG_RPTUN_PRIORITY
---------------------

RPTUN thread priority. Default is 224.

CONFIG_RPTUN_STACKSIZE
----------------------

RPTUN thread stack size. Default is 4096.

Header Files
============

- ``include/nuttx/rptun/rptun.h`` - Main RPTUN header file containing
  all structures, macros, and function prototypes.

Access Macros
=============

The following macros are provided for accessing RPTUN operations:

.. code-block:: c

   /* Get local CPU name */
   RPTUN_GET_LOCAL_CPUNAME(dev)

   /* Get remote CPU name */
   RPTUN_GET_CPUNAME(dev)

   /* Get firmware path */
   RPTUN_GET_FIRMWARE(dev)

   /* Get address environment */
   RPTUN_GET_ADDRENV(dev)

   /* Get resource table */
   RPTUN_GET_RESOURCE(dev)

   /* Check if auto-start is enabled */
   RPTUN_IS_AUTOSTART(dev)

   /* Check if this is the master core */
   RPTUN_IS_MASTER(dev)

   /* Configure remote core */
   RPTUN_CONFIG(dev, data)

   /* Start remote core */
   RPTUN_START(dev)

   /* Stop remote core */
   RPTUN_STOP(dev)

   /* Send notification to remote core */
   RPTUN_NOTIFY(dev, vqid)

   /* Register callback for remote notifications */
   RPTUN_REGISTER_CALLBACK(dev, callback, arg)

   /* Unregister callback */
   RPTUN_UNREGISTER_CALLBACK(dev)

   /* Reset remote core */
   RPTUN_RESET(dev, value)

   /* Trigger panic on remote core */
   RPTUN_PANIC(dev)
