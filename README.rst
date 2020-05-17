========
Overview
========

.. start-badges

.. list-table::
    :stub-columns: 1

    * - docs
      - |docs|
    * - tests
      - | |travis| |requires|
        | |codecov|
    * - package
      - | |version| |wheel| |supported-versions| |supported-implementations|
        | |commits-since|
.. |docs| image:: https://readthedocs.org/projects/python-sa-pathfinding/badge/?style=flat
    :target: https://readthedocs.org/projects/python-sa-pathfinding
    :alt: Documentation Status

.. |travis| image:: https://api.travis-ci.org/nathansamsel/python-sa-pathfinding.svg?branch=master
    :alt: Travis-CI Build Status
    :target: https://travis-ci.org/nathansamsel/python-sa-pathfinding

.. |requires| image:: https://requires.io/github/nathansamsel/python-sa-pathfinding/requirements.svg?branch=master
    :alt: Requirements Status
    :target: https://requires.io/github/nathansamsel/python-sa-pathfinding/requirements/?branch=master

.. |codecov| image:: https://codecov.io/gh/nathansamsel/python-sa-pathfinding/branch/master/graphs/badge.svg?branch=master
    :alt: Coverage Status
    :target: https://codecov.io/github/nathansamsel/python-sa-pathfinding

.. |version| image:: https://img.shields.io/pypi/v/sa-pathfinding.svg
    :alt: PyPI Package latest release
    :target: https://pypi.org/project/sa-pathfinding

.. |wheel| image:: https://img.shields.io/pypi/wheel/sa-pathfinding.svg
    :alt: PyPI Wheel
    :target: https://pypi.org/project/sa-pathfinding

.. |supported-versions| image:: https://img.shields.io/pypi/pyversions/sa-pathfinding.svg
    :alt: Supported versions
    :target: https://pypi.org/project/sa-pathfinding

.. |supported-implementations| image:: https://img.shields.io/pypi/implementation/sa-pathfinding.svg
    :alt: Supported implementations
    :target: https://pypi.org/project/sa-pathfinding

.. |commits-since| image:: https://img.shields.io/github/commits-since/nathansamsel/python-sa-pathfinding/v0.1.svg
    :alt: Commits since latest release
    :target: https://github.com/nathansamsel/python-sa-pathfinding/compare/v0.1...master



.. end-badges

A python utility to execute single-agent searches on environments

* Free software: MIT license

Installation
============

::

    pip install sa-pathfinding

You can also install the in-development version with::

    pip install https://github.com/nathansamsel/python-sa-pathfinding/archive/master.zip


Documentation
=============


https://python-sa-pathfinding.readthedocs.io/


Development
===========

To run the all tests run::

    tox

Note, to combine the coverage data from all the tox environments run:

.. list-table::
    :widths: 10 90
    :stub-columns: 1

    - - Windows
      - ::

            set PYTEST_ADDOPTS=--cov-append
            tox

    - - Other
      - ::

            PYTEST_ADDOPTS=--cov-append tox
