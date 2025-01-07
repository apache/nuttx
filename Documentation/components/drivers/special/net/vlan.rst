===================
Vlan Device Drivers
===================

-  ``include/nuttx/net/vlan.h``. All structures and APIs
   needed to work with Vlan drivers are provided in this
   header file.
-  we also create VLAN devices for managing VLAN, which will become
   interfaces like ``eth0.58``
-  QinQ is also supported, we can create VLAN devices above another VLAN
   devices, like ``eth0.100.101`` (or even ``eth0.1.2.3.4``, also supported
   on Linux).
-  Supporting ADD_VLAN_CMD and DEL_VLAN_CMD of SIOCSIFVLAN.
-  We add default PCP because some of our apps may not want to set
   PCP manually

-  **Driver**: ``drivers/net/vlan.c``
