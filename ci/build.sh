#!/bin/bash
(ls /.dockerenv && echo Found dockerenv) || (echo No dockerenv)
echo "hello world!"
idf.py build
