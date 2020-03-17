#!/bin/bash
pip3 install setuptools_scm
cat requirements.txt | xargs -n1 pip3 install
