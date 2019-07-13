#!/bin/bash
cat requirements.txt | xargs -n1 pip install --ignore-installed --no-cache-dir
