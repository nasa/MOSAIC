.. MOSAIC schedulers documentation master file, created by
   sphinx-quickstart on Sun Jan 19 21:05:44 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

*****************
MOSAIC schedulers
*****************

.. bibliographic fields (which also require a transform):

:Author: Federico Rossi, Tiago Stegun Vaquero, Joshua Vander Hook 
:Address: 4800 Oak Grove Dr.
		  Pasadena, CA 91109
:Contact: federico.rossi@jpl.nasa.gov, tiago.stegun.vaquero@jpl.nasa.gov, hook@jpl.nasa.gov
:Organization: Jet Propulsion Laboratory (JPL)
:Release: |release|
:Repository: https://github.com/nasa/mosaic
:Abstract: Communication-aware dynamic task allocation in multi-robot systems

.. meta::
   :keywords: Multi-robot systems, task allocation, scheduling, planning, planning for robotics

Introduction
============

.. raw:: html
   
   <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
   <iframe src="https://www.youtube-nocookie.com/embed/zTQ7Y4-ax2A" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
   </div>

MOSAIC provides scheduling tools to allocate computational tasks to robots with heterogeneous computational capabilities and time-varying communication links. 

This package contains three+1 scheduling and task allocation algorithm.

- :ref:`time_varying` (``tv_milp``).  A mixed-integer programming algorithm for scheduling tasks in heterogeneous robotic networks with time-varying communication links. The scheduler can accommodate any non-cyclical dependencies between tasks and arbitrary time-varying communication links, handle optional tasks with associated rewards, and optimize cost functions including rewards for optional tasks, makespan, and energy usage. The scheduler is presented in \[1\].

- :ref:`time_invariant`  (``ti_milp``). A mixed-integer programming algorithm for task allocation in  heterogeneous robotic networks with *periodic* communication links. The task allocation algorithm also accommodates any non-cyclical dependencies between tasks and handles optional tasks with associated rewards and maximum latency requirements; it can maximize reward from optional tasks or minimize energy use. The task allocation algorithm is presented in \[2\].

- For benchmarking purposes, we also provide a lightly modified version of Topcuoglu, Hariri, and Wu's `Heterogeneous Earliest Finish Time (HEFT) <#references>`__ algorithm (``heft``).

- A scheduling algorithm (``ti_milp_heft``) that uses the proposed task allocation algorithm to assign tasks to agents, and HEFT (with fixed task allocation) to schedule task execution.

For a full description of the algorithm, we refer the interested reader to `our papers <#references>`__.

Contents
========

.. toctree::
   :maxdepth: 2
   :numbered:

   Installation.rst
   Scheduling.rst
   Allocation.rst
   Examples.rst
   API.md
   class_documentation.rst


Copyright and Licensing
=======================

Copyright (c) 2020, California Institute of Technology ("Caltech").  U.S. Government sponsorship acknowledged.

``mosaic-schedulers`` is currently an open-source package distributed under the Apache 2.0 license.

.. _references:

References
==========

\[1\] Joshua Vander Hook, Tiago Vaquero, Federico Rossi, Martina Troesch, Marc Sanchez Net, Joshua Schoolcraft, Jean-Pierre de la Croix, and Steve Chien, `"Mars On-Site Shared Analytics Information and Computing," <https://aaai.org/ojs/index.php/ICAPS/article/view/3556>`__ in Proceedings of the Twenty-Ninth International Conference on Automated Planning and Scheduling, vol. 29, no. 1, pp. 707-715, July 2019.

\[2\] Federico Rossi\*, Tiago Stegun Vaquero\*, Marc Sanchez Net, Maíra Saboia da Silva, and Joshua Vander Hook, `"The Pluggable Distributed Resource Allocator (PDRA): a Middleware for Distributed Computing in Mobile Robotic Networks," <https://arxiv.org/abs/2003.13813>`__ under review.

\[3\] H. Topcuoglu, S. Hariri and Min-You Wu, `"Performance-effective and low-complexity task scheduling for heterogeneous computing," <https://ieeexplore.ieee.org/document/993206>`__ in IEEE Transactions on Parallel and Distributed Systems, vol. 13, no. 3, pp. 260-274, March 2002.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
