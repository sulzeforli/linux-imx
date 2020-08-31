#!/bin/sh
sudo rm -r /media/one/Boot\ imx6dl/zImage 
sudo rm -r /media/one/Boot\ imx6dl/mx6dsysd.dtb 
sudo cp -r arch/arm/boot/zImage /media/one/Boot\ imx6dl/
sudo cp -r arch/arm/boot/dts/mx6dsysd.dtb /media/one/Boot\ imx6dl/
sync

