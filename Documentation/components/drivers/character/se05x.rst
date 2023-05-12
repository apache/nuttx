============
SE05X Driver
============

This driver enables access to the NXP SE05X secure element by
using the `NXP plug and trust nano <https://github.com/NXPPlugNTrust/nano-package>`_.

.. note::
   Currently this driver has only been tested on SE050.

API
===

The driver supports reading/writing to the SE05X's keystore and additional
features like diffie-hellman key derivation and signing CSR's and verifying
certificates using keys from the keystore.
Refer to ``drivers/crypto/pnt/pnt_se05x_api.h`` for the API functions available
and to ``include/nuttx/crypto/se05x.h`` for the ioctl commands

These tools make use of the SE05X driver (which can function as a reference project):

- The ``controlse`` app can be used to control the SE05X from NSH.

- The ``setest`` app tests all the SE05X ioctl functionality from NSH

Datasheets are available on the `NXP website <https://www.nxp.com/products/security-and-authentication/authentication/edgelock-se050-plug-trust-secure-element-family-enhanced-iot-security-with-high-flexibility:SE050>`_.

Configuration
=============

- ``DEV_SE05X`` Enable support for /dev/se05x secure element provided by NXP SE050
  or SE051

  - Channel communication interface

    - ``DEV_SE05X_SCP03`` SCP03 secure channel (Not implemented)

      - ``DEV_SE05X_SCP03_KEY_FILE`` Specify file containing the keys needed with
        SCP03 channel authentication.
        Location may be relative to the NuttX root folder. File should contain
        the definitions for SCP03_ENC_KEY, SCP03_MAC_KEY and SCP03_DEK_KEY as
        byte array initializers.

    - ``DEV_SE05X_PLAIN`` plain communication

  - ``SE05X_LOG_LEVEL`` The SE05x log is divided into the following levels: ERROR,WARNING,INFO,DEBUG.
