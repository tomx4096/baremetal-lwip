# Baremetal LwIP
This is a port of the LwIP to an ARM board without an operating system. The target machine (VersatilePB) has an ARM926EJ-S processor and LAN91C111 ethernet controller. It exists mostly because I couldn't find a working NO_SYS==1 port, but is also a good starting point for building and learning new things ~~*~

# Setup
The intended execution environment is qemu-system-arm on a Linux host. To build:
```
git clone https://github.com/tomx4096/baremetal-lwip.git
cd baremetal-lwip
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
tar xvjf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
mv gcc-arm-none-eabi-7-2017-q4-major-linux gcc
rm gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
mkdir bin
make
```
Next, use this script to bring up a TAP interface to create a bridge between Linux and QEMU's network interfaces. Change the ethernet interface name and settings in the script to match yours.
```
sudo ./qemu-ifup tap0
```
Then bring up the qemu machine (requires root to use the qemu tap backend)
```
sudo make run
```
Now you can talk to it from the Linux host:
```console
$ ping 10.0.2.99
PING 10.0.2.99 (10.0.2.99) 56(84) bytes of data.
64 bytes from 10.0.2.99: icmp_seq=1 ttl=255 time=4.41 ms
64 bytes from 10.0.2.99: icmp_seq=2 ttl=255 time=0.549 ms
64 bytes from 10.0.2.99: icmp_seq=3 ttl=255 time=0.528 ms
```
Exit qemu (Ctrl-A X) and bring down the bridge when finished:
```
sudo ./qemu-ifdown2
```

# TODO
* Target a more modern board and eventually real hardware..versatilepb was chosen because it is used in many QEMU tutorials
* Add an abstraction layer so swapping ethernet drivers is cleaner
* Implement system timer

# Sources
* [LAN91C111 Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/00002276A.pdf) 
* [VersatilePB Reference](https://static.docs.arm.com/dui0225/d/DUI0225D_versatile_application_baseboard_arm926ej_s_ug.pdf)
* [Building bare-metal ARM with GNU](https://www.mikrocontroller.net/attachment/66084/Building_bare-metal_ARM_with_GNU.pdf)
* [LwIP for bare metal](http://lwip.wikia.com/wiki/Porting_For_Bare_Metal)
* [C standard libary on bare metal](https://balau82.wordpress.com/2010/12/16/using-newlib-in-arm-bare-metal-programs/)
* [LAN91C111 Driver Source](http://www.jk1mly.org/electoronics/nios/) -- a real gem
* [Connecting QEMU to a Real Network](https://emreboy.wordpress.com/2012/12/24/connecting-qemu-to-a-real-network/)

