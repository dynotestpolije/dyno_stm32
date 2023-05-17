[<img alt="github" src="https://img.shields.io/badge/github-dynotestpolije/dyno_stm32-8da0cb?logo=github" height="20">](https://github.com/dynotestpolije/dyno_stm32)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/dynotestpolije/dyno_types/blob/master/LICENSE)

<center>
    <h1>Dynotest Firmware</h1>
    <p>Firmware microcontroller STM32F411CEU6 BlackPill for dynotest by Rizal Achmad Pahlevi</p>
</center>


## INSTALATIONS
#### DEPENDENCIES

**(Debian / Ubuntu)**
```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa && sudo apt-get update;
sudo apt-get install make which findutils git build-essential libusb-1.0.0-dev cmake \
    gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi;
curl -fsSLO 'https://github.com/stlink-org/stlink/releases/download/v1.7.0/stlink_1.7.0-1_amd64.deb';
sudo dpkg -i ./stlink_1.7.0-1_amd64.deb;
```

**(Arch)**
```bash
sudo pacman -S gcc make findutils arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-binutils \
    stlink libusb libtool pkg-config autoconf automake git which;
```

#### BUILD FROM SOURCE
**LINUX**
1. install dependencies [DEPENDENCIES](#dependencies)
2. clone or download the repo
```bash 
git clone --depth=1 'https://github.com/dynotestpolije/dyno_stm32.git'
cd dyno_app
```
3. make sure the device (STM32F411CEU6) is connected to the host, using stlink or other way to
   upload/flash the stm32 arm firmware
4. build the firmware
```bash 
./tools/build.sh 
```
5. upload the firmware
```bash
./tools/upload.sh
```
5. finish


