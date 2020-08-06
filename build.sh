#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=5.4.1-arm-linux-gnueabihf-
make distclean
make imx_mx6dsysd_defconfig
#make imx_v7_defconfig
make menuconfig
make all -j8
make zImage

