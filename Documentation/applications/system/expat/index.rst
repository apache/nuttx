============================
``expat`` XML parser library
============================

``CONFIG_LIB_EXPAT`` provides the Expat streaming XML parser as a reusable
NuttX applications library.

General entity expansion and DTD processing are disabled by default.  Enable
``CONFIG_LIB_EXPAT_GENERAL_ENTITIES`` or ``CONFIG_LIB_EXPAT_DTD`` only when an
application requires them and its XML input policy has been reviewed.

Expat uses the MIT license, so ``CONFIG_ALLOW_MIT_COMPONENTS`` must be enabled.
