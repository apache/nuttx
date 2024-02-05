================================
``smart_test`` SMART File System
================================

``CONFIG_TESTING_SMART_TEST=y``

Author: Ken Pettit
Date: April 24, 2013

Performs a file-based test on a SMART (or any) filesystem. Validates seek,
append and seek-with-write operations::

  Usage:

    flash_test mtdblock_device

  Additional options:

    --force                     to replace existing installation

