pro3-firmware
=============

### Setup

Requires an arm-none-eabi- set of gcc tools

    git clone --recursive https://github.com/noah95/ekg3000-firmware.git
    git submodule update --init --recursive

### Build

    cd ekg3000-firmware/
    make

### Program

Using [stlink](https://github.com/texane/stlink)

    {sudo} st-flash write build/stm32f3discovery-demo.bin 0x8000000

### Use

Connect to the USB USER port.

    {sudo} cat /dev/ttyACM0

### Debug

Using [stlink](https://github.com/texane/stlink).

In one terminal:

    {sudo} st-util
    
And another:

    arm-non-eabi-gdb build/stm32f3discovery-demo.elf
    
And then within GDB:

    > target extended-remote :4242
    ...
    > load
    ...
    
And you can debug with GDB as you would expect.

### Toolchain installation under ubuntu linux

    sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
    sudo apt-get update
    sudo apt install gcc-arm-none-eabi
