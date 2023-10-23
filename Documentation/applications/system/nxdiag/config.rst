.. |br| raw:: html

   <br/>

======================
Configuration Settings
======================

The availability of the above commands depends upon features that may or
may not be enabled in the NuttX configuration file. The following
:ref:`cmdtable <nxdiagcmddependencies>` indicates the dependency of each command on
NuttX configuration settings. General configuration settings are
discussed in the NuttX Porting Guide.
Configuration settings specific to Nxdiag as discussed at the
:ref:`cmdbottom <nxdiagconfiguration>` of this document.

Note that the ``--vendor-specific`` or ``-v`` option will generate vendor-specific
information and checks. The output of this option will depend on the selected
vendors in the NuttX configuration file. For example, if the ``CONFIG_SYSTEM_NXDIAG_ESPRESSIF``
configuration setting is enabled, then this option will provide custom
information and checks for Espressif devices. Multiple vendors may be selected
at the same time.

.. _nxdiagcmddependencies:

Option Dependencies on Configuration Settings
=============================================

========================= ===========================================
Option                    Depends on Configuration
========================= ===========================================
``--help, -h``
``--nuttx, -n``
``--flags, -f``           ``CONFIG_SYSTEM_NXDIAG_COMP_FLAGS``
``--config, -c``          ``CONFIG_SYSTEM_NXDIAG_CONF``
``--host-os, -o``
``--host-path, -p``       ``CONFIG_SYSTEM_NXDIAG_HOST_PATH``
``--host-packages, -k``   ``CONFIG_SYSTEM_NXDIAG_HOST_PACKAGES``
``--host-modules, -m``    ``CONFIG_SYSTEM_NXDIAG_HOST_MODULES``
``--vendor-specific, -v``
``--all``

========================= ===========================================

.. _nxdiagconfiguration:

Nxdiag-Specific Configuration Settings
======================================

The behavior of Nxdiag can be modified with the following settings in the
``boards/<arch>/<chip>/<board>/defconfig`` file:

========================================  ==================================
Configuration                             Description
========================================  ==================================
 ``CONFIG_SYSTEM_NXDIAG_COMP_FLAGS``      Enable the nxdiag application to list the NuttX compilation
                                          flags. This is useful for debugging the host and target
                                          systems. Enables the ``-f`` and ``--nuttx-flags`` options.

 ``CONFIG_SYSTEM_NXDIAG_CONF``            Enable the nxdiag application to list the configuration options
                                          used to compile NuttX. This is useful for debugging the host and
                                          target systems. Enables the ``-c`` and ``--nuttx-config`` options.

 ``CONFIG_SYSTEM_NXDIAG_HOST_PATH``       Enable the nxdiag application to list the host system PATH
                                          variable. This is useful for debugging the host system.
                                          Enables the ``-p`` and ``--host-path`` options.

 ``CONFIG_SYSTEM_NXDIAG_HOST_PACKAGES``   Enable the nxdiag application to list the installed packages
                                          on the host system. This is useful for debugging the host
                                          system. Enables the ``-k`` and ``--host-packages`` options.

 ``CONFIG_SYSTEM_NXDIAG_HOST_MODULES``    Enable the nxdiag application to list the installed Python
                                          modules on the host system. This is useful for debugging the
                                          host system. Enables the ``-m`` and ``--host-modules`` options.

 ``CONFIG_SYSTEM_NXDIAG_ESPRESSIF``       Enable Espressif-specific information and checks.

========================================  ==================================
