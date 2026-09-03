#!/bin/bash

rm -rf build
rm -rf bin
mkdir build
mkdir bin

cd build
cmake "../" || exit 1
make
cp -rf "./bin" "../"
cd ".."
