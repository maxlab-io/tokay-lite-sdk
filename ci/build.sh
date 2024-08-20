#!/bin/bash
# TODO: print more information about the container

(ls /.dockerenv && echo Found dockerenv) || (echo No dockerenv)
echo "IDF: $IDF_PATH"
echo "PATH: $PATH"

# why full path if the container must provide idf.py right away?
/opt/esp/idf/tools/idf.py build
