#!/bin/bash
rm -rf build && \
mkdir build && \
cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev && \
make -j$(sysctl -n hw.physicalcpu)