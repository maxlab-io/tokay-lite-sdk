#!/bin/bash

# TODO: print more information about the container

(ls /.dockerenv && echo Found dockerenv) || (echo No dockerenv)
echo "IDF_PATH: $IDF_PATH"
echo "PATH: $PATH"

. "${IDF_PATH}/export.sh"
idf.py build
