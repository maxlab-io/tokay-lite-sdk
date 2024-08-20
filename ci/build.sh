#!/bin/bash
# TODO: print more information about the container

(ls /.dockerenv && echo Found dockerenv) || (echo No dockerenv)
echo "IDF: $IDF_PATH"
echo "IDF_PATH: $IDF_PATH"
echo "PATH: $PATH"

source "$IDF/export.sh"
idf.py build
