.. include:: /substitutions.rst

.. note:: This is a first version of the document, will be updated once new documentation
  system is in place.

Documentation
=============

The Apache NuttX Documentation is made using the
`Sphinx documentation system <https://www.sphinx-doc.org/en/master/>`_. Sphinx documentation
is written in `ReStructured Text <https://docutils.sourceforge.io/rst.html>`_ (RST). RST is
the format used for `Python documentation <https://docs.python.org/3/>`_ and is also used
in many other projects.

Contributions and fixes to the Apache NuttX Companion are encouraged and welcome. Here's how to do
it.

#. Fork the Apache NuttX Documentation Repository

   Visit this link and hit the Fork button in the upper right of the page:

   * `NuttX (v01d's fork) <https://github.com/v01d/incubator-nuttx/>`_

   |br|

#. Clone the Forked Repository

   Click the "Clone or Download" button and copy the clone URL. Then do this:

    .. code-block:: bash

       $ git clone <CLONE-URL-THAT-YOU-COPIED>

#. Install Sphinx

    On the command line, change directories to the newly-downloaded repository directory:

    .. code-block:: bash

       $ cd incubator-nuttx
       $ # install pyenv
       $ curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash
       $ # install python
       $ pyenv install 3.7.3
       $ pyenv local 3.7.3
       $ # install pipenv
       $ pip install pipenv
       $ # install sphinx and related software
       $ pipenv install
       $ # activate the virtual environent
       $ pipenv shell

#. Build the HTML Documentation Locally

    .. code-block:: bash

       $ cd docs
       $ make html
       Running Sphinx v2.4.1
       making output directory... done
       building [mo]: targets for 0 po files that are out of date
       building [html]: targets for 12 source files that are out of date
       updating environment: [new config] 12 added, 0 changed, 0 removed
       reading sources... [100%] user/simulator
       looking for now-outdated files... none found
       pickling environment... done
       checking consistency... done
       preparing documents... done
       writing output... [100%] user/simulator
       generating indices...  genindexdone
       writing additional pages...  searchdone
       copying static files... ... done
       copying extra files... done
       dumping search index in English (code: en)... done
       dumping object inventory... done
       build succeeded.

       The HTML pages are in ``_build/html``.

#. Check Out a New Branch

    .. code-block:: bash

       $ git checkout -b feature/my-branch

#. Make Your Changes

    .. code-block:: bash

       $ vim user/intro.rst  # or use the editor of your choice
                                  # on whatever file you want to change

#. Rebuild the HTML Documentation

    .. code-block:: bash

       $ make html

#. View the Documentation in a Web Browser

   You can open the file ``docs/_build/html/index.html``.
   |br|
   |br|

#. Iterate

   Repeat Steps 6, 7, and 8 until you're happy with the result.
   |br|
   |br|

#. Commit the Changes

    .. code-block:: bash

       $ git add introduction/main.rst  # or whatever files you changed
       $ git commit

#. Push to Your Branch

    .. code-block:: bash

       $ git push

#. Submit Your Pull Request

   Go to the `Apache NuttX (v01d's fork) <https://github.com/v01d/incubator-nuttx/>`_ page
   on Github and click the "Pull Request" button. See Github's `Creating a Pull Request
   <https://help.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request>`__
   page for more info.

   Use this template for the Pull Request description text:

    ::

       ### Summary
       ### Impact
       ### Limitations / TODO
       ### Detail
       ### Testing
       ### How To Verify

   Fill out the sections describing your changes. The summary should be a concise bulleted
   list.

   The ``How To Verify`` section is only needed if you change how the project is built or
   add some other programs or scripts.
   |br|
   |br|

#. Make Changes If Requested

   When you submit your Pull Request, the Apache NuttX Documentation team will review the changes,
   and may request that you modify your submission. Please work with them to get your
   changes accepted.
   |br|
   |br|

#. You're Done!

   Feel good that you've made Apache NuttX documentation better for yourself and others. You've
   just made the world a better place!

Sphinx Resources
----------------

* `Sphinx documentation system <https://www.sphinx-doc.org/en/master/>`__
* `ReStructured Text documentation <https://docutils.sourceforge.io/rst.html>`__
* `Sphinx Guide to ReStructured Text <http://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`__
* `Restructured Text cheat sheet <https://thomas-cokelaer.info/tutorials/sphinx/rest_syntax.html>`__

