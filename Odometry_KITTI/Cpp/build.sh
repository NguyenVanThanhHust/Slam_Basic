#!/bin/bash

cmake .
make 
rm cmake_install.cmake CMakeCache.txt Makefile
rm -rf CMakeFiles