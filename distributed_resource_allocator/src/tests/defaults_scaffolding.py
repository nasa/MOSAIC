#!/usr/bin/python

'''
# =====================================================================
# Default values for testing purposes
#
# Author: Marc Sanchez Net
# Date:   02/06/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

import os
import sys

# Hack for importing files from other function
def patch_imports():
    _pdra_dir = os.path.abspath(os.path.join(__file__,os.pardir,os.pardir,'pdra'))
    sys.path.append(_pdra_dir)

# Rate for publishing topic [sec]
PUB_RATE = 1

# Queue size for a publisher
PUB_QUEUE_SIZE = 100