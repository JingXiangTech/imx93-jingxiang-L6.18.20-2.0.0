#!/bin/bash
set -e

export ARCH=arm64
export CROSS_COMPILE=aarch64-poky-linux-

# make distclean
make jingxiang_defconfig
# make imx_v8_defconfig 
# make dtbs
make -j"$(nproc)"
# make -j8

 