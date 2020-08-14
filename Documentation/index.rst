===================
NuttX Documentation
===================

NuttX is a real-time operating system (RTOS) with an emphasis on standards compliance and small footprint. Scalable from 8-bit to 32-bit microcontroller environments, the primary governing standards in NuttX are Posix and ANSI standards. Additional standard APIs from Unix and other common RTOS’s (such as VxWorks) are adopted for functionality not available under these standards, or for functionality that is not appropriate for deeply-embedded environments (such as fork()).

.. warning::
   These pages are meant to be an experiment in replacing the `Apache NuttX Documentation <https://cwiki.apache.org/confluence/display/NUTTX/Nuttx>`_
   with something easier to navigate, extend, and modify.
   This is a work in progress, some formatting issues may still be present and are being worked on. Also, some links may be broken. Please
   refer to :doc:`contributing documentation<contributing/documentation>` to propose fixes/improvements.

Last Updated: |today|

.. toctree::
   :caption: Contents:
   :maxdepth: 2

   Home <self>
   introduction/index.rst
   quickstart/index.rst
   reference/index.rst
   components/index.rst
   applications/index.rst
   boards/index.rst
   guides/index.rst
   releases/index.rst
   contributing/index.rst
   glossary.rst
   
.. include:: substitutions.rst
