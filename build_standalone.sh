#!/bin/bash
set -e

export ARCH=arm64
#export CROSS_COMPILE=aarch64-linux-gnu-
export CROSS_COMPILE=aarch64-poky-linux-

# make distclean
# make hf81x_defconfig
# make menuconfig
# make savedefconfig



make distclean
make imx_v8_defconfig 
# make dtbs
make -j"$(nproc)"

