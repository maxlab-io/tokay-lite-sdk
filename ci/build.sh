#!/bin/bash

# TODO: print more information about the container

VERSTR=$(git describe --tags --dirty)

(ls /.dockerenv && echo Found dockerenv) || (echo No dockerenv)
echo "IDF_PATH: $IDF_PATH"
echo "PATH: $PATH"
echo "VERSTR $VERSTR"

. "${IDF_PATH}/export.sh"
idf.py -C webserver build
cd webserver/build

echo "Generating single binary..."
# TODO: use VERSTR to encode version info into the file name
# and use it in the release.yml CI file
esptool.py --chip ESP32-S3 merge_bin -o tokay_lite.bin @flash_args
