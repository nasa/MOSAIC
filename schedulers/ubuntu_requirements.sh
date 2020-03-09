#!/bin/bash
apt-get -y install pkg-config git
# SCIP 
apt-get -y install python-tk cmake coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev doxygen libbz2-dev bison flex libgsl-dev libgfortran3
apt-get -y install libgmp-dev
# GLPK
apt-get -y install glpk-utils libglpk-dev glpk-doc
# Matplotlib
apt-get -y install libfreetype6-dev libpng-dev python3-tk
