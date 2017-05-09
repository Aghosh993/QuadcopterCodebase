#!/bin/bash
echo -e "o\nn\np\n1\n4096\n\nw" | fdisk $1
mkfs.ext4 $1p1
mkdir root
mount $1p1 root
wget http://os.archlinuxarm.org/os/ArchLinuxARM-odroid-xu3-latest.tar.gz
tar -xvf ArchLinuxARM-odroid-xu3-latest.tar.gz -C root
cd root/boot
sh sd_fusing.sh $1
cd ../..
umount root
sync
rm -rf root
rm ArchLinuxARM-odroid-xu3-latest.tar.gz
echo "Done!!"
