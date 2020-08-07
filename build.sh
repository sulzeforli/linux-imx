#!/bin/sh
. /opt/fsl-imx-xwayland/4.14-sumo/environment-setup-cortexa9hf-neon-poky-linux-gnueabi
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-
make distclean
make imx_mx6dsysd_defconfig
#make imx_v7_defconfig
#make menuconfig
make all -j8
make zImage
make dtbs

