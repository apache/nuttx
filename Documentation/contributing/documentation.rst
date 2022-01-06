=============
Documentation
=============

The Apache NuttX Documentation is built using the
`Sphinx documentation system <https://www.sphinx-doc.org/en/master/>`_. Documentation
is written in `ReStructured Text <https://docutils.sourceforge.io/rst.html>`_ (RST),
with Sphinx-specific directives. RST is the format used for
`Python documentation <https://docs.python.org/3/>`_ and is also used in many other projects.
Using Sphinx, the RST files are rendered into HTML files that can be read in your browser.

Building
========

To render the Documentation locally, you should clone the NuttX main repository, and
go into ``Documentation`` directory. Then,

  1. Install Sphinx and other dependencies using pipenv.
     You may also find it helpful on platforms such as Windows and MacOS to use *pyenv*
     to manage your python installation.  You can read about installing that on the
     project `site <https://github.com/pyenv/pyenv#installation>`_.

    .. code-block:: console

      $ pip3 install pipenv
      $ pipenv install
      $ # activate the virtual environment
      $ pipenv shell

  2. Build documentation:

    .. code-block:: console

      $ make html

    The resulting HTMLs will end up under ``_build/html``. You can open your browser at the root with:

    .. code-block:: console

      $ xdg-open _build/html/index.html

Live Rebuild
------------

For more comfortable editing and previewing of changes (as ``make html`` will perform a slower full rebuild),
you can install ``sphinx-autobuild`` which will monitor file changes and rebuild only affected files. To
install it (within the virtual environment):

.. code-block:: console

  $ pip3 install sphinx-autobuild

To run:

.. code-block:: console

  $ make autobuild

Which will perform an initial clean build and monitor changes from then on.

Contributing
============

Contributions to documentation are appreciated. These can be as simple as fixing a typo or formatting issues to more involved
changes such as documenting parts of NuttX which are not yet covered or even writing guides for other users.

The contribution workflow is the same as for the code, so check the :doc:`/contributing/workflow` to understand
how your changes should be upstreamed.

Writing ReStructure Text with Sphinx
====================================

The following links can be used to learn about RST syntax and about Sphinx specific directives. Note that
sometimes Sphinx's approach is used over standard RST since it is more powerful (e.g. standard linking vs Sphinx
``:ref:`` which can be used across files, ``code-block`` directive vs ``::`` which allows specifying highlight language, etc.):

  * `Sphinx documentation system <https://www.sphinx-doc.org/en/master/>`__
  * `ReStructured Text documentation <https://docutils.sourceforge.io/rst.html>`__
  * `Sphinx Guide to ReStructured Text <http://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`__
  * `Restructured Text cheat sheet <https://thomas-cokelaer.info/tutorials/sphinx/rest_syntax.html>`__

Documentation Conventions
=========================

While RST/Sphinx provide many ways to do things, it is best to follow a given convention to maintain consistency and avoid
pitfalls. For this reason, documentation changes should follow the following set of conventions.

Indentation
-----------

Child blocks should be indented two-spaces. This includes itemizations/enumerations.

Headings
--------

Three levels of headings should be used in general. The style used to mark sections is based around ``=`` and ``-``.
Sections should look like this:

.. code-block:: RST

  =================
  Top Level Heading
  =================

  Subsection
  ==========

  Subsubsection
  -------------

Code
----

Code should be documented using the `C domain <https://www.sphinx-doc.org/en/master/usage/restructuredtext/domains.html#the-c-domain>`_.
This means for example that a function should be documented as:

.. code-block:: RST

  .. c:function:: bool myfunction(int arg1, int arg2)

    Here the function should be described

    :param arg1: Description of arg1
    :param arg2: Description of arg2

    :return: Description of return value

To document a piece of code, use a ``code-block`` `directive <https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html#directive-code-block>`_, specifying the highlight language. If the block is not of code but some verbatim piece of text,
it is acceptable to use RST standard `::`. This is specially useful and compact when used in the following mode:

.. code-block:: RST

  The text file should have the following content::

    Line1
    Line2
    Line3

Linking
-------

To generate internal links, Sphinx's `roles <https://www.sphinx-doc.org/en/master/usage/restructuredtext/roles.html#ref-role>`_ should
be used. So, use ``:ref:`` instead of standard RST syntax like ```link <target>`_`` for internal links.
If the target is in a different file, you can refer it with: ``:ref:`link text </pathtorst:Section Name>```.

Linking to a specific document can be done with ``:doc:`/path/to/document``` (without ``.rst`` extension).

Notes and TODOS
---------------

Use RST `admonitions <https://docutils.sourceforge.io/docs/ref/rst/directives.html#admonitions>`_ to highlight things from the text,
such as a note that should be prominently displayed.

In case you need to leave a TODO note in the documentation to point that something needs to be improved, use a ``todo`` admonition,
which is available via the ``sphinx.ext.todo`` extension. This will let the reader of the documentation also know that the documentation
is not yet finished somewhere and may further motivate a contribution.

User Indications
----------------

To indicate a keypress, menu action or GUI button selection, use the following:

.. code-block:: RST

  Go into menu :menuselection:`File --> Save As`, click :guilabel:`&OK` or press :kbd:`Enter`.

which would render as:

Go into menu :menuselection:`File --> Save As`, click :guilabel:`&OK` or press :kbd:`Enter`.

Tabbed examples
---------------

To indicate different instructions/examples for different scenarios (for example, different Operating
Systems) use the `tabs <https://github.com/executablebooks/sphinx-tabs>`_ extension (see link for examples).

Tips
====

Spacing
-------

If you are getting formatting errors, be sure to provide the appropriate spacing between a directive and its content.
Generally, you should follow this format:

.. code-block:: RST

  .. directive::

    child content

  non-child content which appears after previous directive

Note the line between directive and content and the indentation.

