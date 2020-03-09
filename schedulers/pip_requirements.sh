#!/bin/bash
cat requirements.txt | xargs -n1 pip3 install --ignore-installed --no-cache-dir
