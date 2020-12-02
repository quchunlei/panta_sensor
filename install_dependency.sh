#!/bin/bash

#######################################################################################################
echo -e "\033[32m =====>             Installing Dependancy               <===== \033[0m"

## install basic third-part libraries ##

#install libpcap
sudo apt-get install -y  libpcap-dev

#install openssl
sudo apt-get install -y openssl
sudo apt-get install -y libssl-dev

#install protobuf
sudo apt-get install -y libprotobuf-dev protobuf-compiler

#install wireshark
sudo apt-get install -y wireshark

#install doxygen
sudo apt-get install -y doxygen

#install build essential
sudo apt-get install -y build-essential git cmake dmidecode

#install qt-base
sudo apt-get install -y freeglut3-dev qtbase5-dev libqt5opengl5-dev libglew-dev

#install auto configure tool
sudo apt-get install -y autoconf automake libtool

#install key driver
sudo cp 80-hasp.rules /etc/udev/rules.d/
