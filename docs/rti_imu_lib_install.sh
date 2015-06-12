#!/usr/bin/env sh

install_dependencies=${1:-true}
clone_git_repository=${2:-true}
make_and_build_rtimulib=${3:-true}
library_path=${4:-~/}



if [ "${install_dependencies}" = true ]; then
   echo "\n\n"
   echo "####################################################################################################"
   echo "##### Installing dependencies for RTIMULib"
   echo "####################################################################################################"

   sudo apt-get install cmake
   sudo apt-get install libqt4-dev
   sudo apt-get install python-dev
   sudo apt-get install octave
   sudo apt-get install i2c-tools

   #sudo raspi-config
   #sudo adduser pi i2c
   #i2cdetect -y 1
fi



if [ "${clone_git_repository}" = true ]; then
   echo "\n\n"
   echo "####################################################################################################"
   echo "##### Cloning git RTIMULib repository"
   echo "####################################################################################################"

   cd ${library_path}
   git clone https://github.com/richards-tech/RTIMULib.git

   echo "\n\n"
   echo "----------------------------------------------------------------------------------------------------"
   echo ">>>>> Cloning finished"
   echo ">>>>> For updating git repository use: git pull"
   echo "----------------------------------------------------------------------------------------------------"
fi



if [ "${make_and_build_rtimulib}" = true ]; then
   echo "\n\n"
   echo "####################################################################################################"
   echo "##### Building RTIMULib"
   echo "####################################################################################################"

   mkdir -p ${library_path}RTIMULib/Linux/build
   cd ${library_path}RTIMULib/Linux/build
   cmake ..
   make

   echo "\n\n"
   echo "----------------------------------------------------------------------------------------------------"
   echo ">>>>> Build of RTIMULib finished"
   echo "----------------------------------------------------------------------------------------------------"



   echo "\n\n"
   echo "####################################################################################################"
   echo "##### Installing RTIMULib"
   echo "####################################################################################################"

   sudo make install
   sudo ldconfig  

   cd ${library_path}RTIMULib/Linux/python/
   python setup.py build
   sudo python setup.py install

   echo "\n\n"
   echo "----------------------------------------------------------------------------------------------------"
   echo ">>>>> Install of RTIMULib finished"
   echo "----------------------------------------------------------------------------------------------------"
   echo "\n\n"
fi
