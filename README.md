# LwIP 

# Setup

```
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
tar xvjf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
mv gcc-arm-none-eabi-7-2017-q4-major-linux gcc
rm gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
mkdir bin
make
sudo make run
```
This will bring up a host in qemu that you can talk to:
```console
$ ping 10.0.2.99
PING 10.0.2.99 (10.0.2.99) 56(84) bytes of data.
64 bytes from 10.0.2.99: icmp_seq=1 ttl=255 time=4.41 ms
64 bytes from 10.0.2.99: icmp_seq=2 ttl=255 time=0.549 ms
64 bytes from 10.0.2.99: icmp_seq=3 ttl=255 time=0.528 ms
```

# TODO

# Sources
