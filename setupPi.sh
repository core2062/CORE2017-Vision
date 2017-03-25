#!/bin/bash

#To run:
#wget -O setupPi.sh https://raw.githubusercontent.com/core2062/CORE2017-Vision/master/setupPi.sh && sudo chmod 775 setupPi.sh
#sudo ./setupPi.sh

#Set locale
sudo echo "en_US.UTF-8 UTF-8" > /etc/locale.gen

#Install required packages
sudo apt-get purge wolfram-engine -y
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install git build-essential cmake vim pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev \
                         libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
                         libgtk2.0-dev libatlas-base-dev gfortran python2.7-dev python3-dev screen feh cmake-curses-gui -y

#Download and compile OpenCV
cd /home/pi
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.2.0.zip
unzip opencv.zip
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
pip install numpy
cd opencv-3.2.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig
rm -rf cd /home/pi/opencv.zip

#Download and compile CORE Vision code
cd /home/pi
git clone https://github.com/core2062/CORE2017-Vision
install -d -m 700 /home/pi/.ssh
cd CORE2017-Vision
sudo cp authorized_keys /home/pi/.ssh/authorized_keys
chmod 600 /home/pi/.ssh/authorized_keys
sudo chmod +x launch.sh
sudo chmod +x pitDisplay/pitDisplay.sh
sudo cp interfaces /etc/network/interfaces
sudo cp CORE2017-Vision.service /lib/systemd/system/CORE2017-Vision.service
sudo cp pitDisplay/COREPitDisplay.service /lib/systemd/system/COREPitDisplay.service
sudo systemctl daemon-reload
sudo systemctl enable CORE2017-Vision.service
#sudo systemctl enable COREPitDisplay.service
git clone https://github.com/core2062/COREVisionLib

mkdir build
cd build
cmake ..
make -j4

#Setup pit display
sudo sed -ie 's/BLANK_TIME=30/BLANK_TIME=0/g' /etc/kbd/config
sudo sed -ie 's/POWERDOWN_TIME=30/POWERDOWN_TIME=0/g' /etc/kbd/config
cd /home/pi
wget -O orbitron.zip https://fonts.google.com/download?family=Orbitron
unzip orbitron.zip
sudo cp Orbitron-*.ttf /usr/share/fonts/
sudo echo "@/home/pi/CORE2017-Vision/pitDisplay/pitDisplay.sh" >> /home/pi/.config/lxsession/LXDE-pi/autostart


# Follow this guide: https://pimylifeup.com/raspberry-pi-photo-frame/
