************
Installation
************

Requirements
============

Libraries
---------


The scheduler depends on the GLPK_ (MI)LP solver.
GLPK and its dependencies can be installed by running ``ubuntu_requirements.sh`` (on Debian and derivatives, including Ubuntu) or ``fedora_requirements.sh`` (on Fedora and derivatives, including CentOS). This will also install some of the requirements to compile the SCIP solver from source (see below).

If available, the scheduler can also make use of the following commercial solvers, which both offer academic licensing options:

- SCIP_
- CPLEX_

SCIP provides `RPM and Deb binaries`_ for x86. On ARM, we recommend compiling from source (see below).
Place the SCIP source code in a folder (e.g., ``scipoptsuite-6.0.1``), and then run:

.. w

   cd scipoptsuite-6.0.1
   mkdir build
   cd build
   cmake ..
   make
   make install

If CPLEX is not available, the solvers will fall back to GLPK.
An academic license for CPLEX can be obtained from IBM_ after registering for the `IBM Academic Initiative`_. Using the academic license is acceptable for non-commercial research: specifically, _"Noncommercial research purposes are defined as conducting not-for-profit research projects whose results would be considered to be in the public domain and suitable for submission to a peer-reviewed journal or conference for publication. IBM Products may be used in noncommercial research that is focused on the business concepts, science, math or technology upon which the product is based."_ (per the `IBM Academic Initiative Software Usage guidelines`_, retrieved on 2019-7-10).


OPTIC solver binaries
---------------------
Binaries for the `OPTIC planner`_ for x64, armhf, and ppc32 are available in at ``mosaic_schedulers/schedulers/pddl/planners/optic-clp``.

OPTIC can also be compiled from source by following `these instructions`_.

.. warning::
    If compiling OPTIC from source, make sure to download the most recent version of Optic (with the GCC 8 patch) from here. Do **not** follow the direct download link on the installation guide.

.. warning::
    If compiling OPTIC from source, add ``-llapack -lblas -lgfortran pthread`` at the end of the linked libraries on line 60 of ``src/optic/CMakeLists.txt``.


Python dependencies
-------------------

Python dependencies can be installed by running the script ``pip_requirements.sh``.

Installation
============

To install the package in pip, cd to the root folder of the repository and run 

.. code-block:: bash
   
   pip install .

Docker
======

Dockerfiles are provided to package the MOSAIC schedulers in a Docker container. The Docker container exposes a web server that receives requests containing MOSAIC problems (in JSON format) and returns the computed schedules.

The Docker image is designed to use the SCIP scheduler and compile it from source. The SCIP source code should be placed in ``docker_resources/scipoptsuite-6.0.1``.

To build the MOSAIC docker image, run

.. code-block:: bash

   docker build --tag=mosaic/schedulers_requirements -f Dockerfile_requirements .
   docker build --tag=mosaic/schedulers -f Dockerfile .


When changes are made to the ``mosaic_schedulers`` package, only the second Docker image needs to be rebuilt.

To run the Docker image and expose the MOSAIC service on the local network,

.. code-block:: bash

   docker run --network="host" mosaic/schedulers


The service exposes an instance of ``MOSAIC_server.py`` at ``localhost:4000``. The web interface accepts PUT requests containing :doc:`a JSON description of the problem </API>` and returns computed schedules.


.. _GLPK: https://www.gnu.org/software/glpk/
.. _SCIP: https://scip.zib.de
.. _CPLEX: https://www.ibm.com/products/ilog-cplex-optimization-studio
.. _RPM and Deb binaries: https://scip.zib.de/index.php#download
.. _IBM: https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=613c3d21-0ce1-e711-80fa-000d3af41938
.. _IBM Academic Initiative: https://my15.digitalexperience.ibm.com/b73a5759-c6a6-4033-ab6b-d9d4f9a6d65b/dxsites/151914d1-03d2-48fe-97d9-d21166848e65/home
.. _IBM Academic Initiative Software Usage guidelines: https://my15.digitalexperience.ibm.com/b73a5759-c6a6-4033-ab6b-d9d4f9a6d65b/dxsites/151914d1-03d2-48fe-97d9-d21166848e65/faqs/guidelines
.. _OPTIC planner: https://nms.kcl.ac.uk/planning/software/optic.html
.. _these instructions: https://nms.kcl.ac.uk/planning/software/optic.html
.. _here: https://sourceforge.net/projects/tsgp/files/OPTIC/optic-patched-for-gcc8.tar.bz2/download
