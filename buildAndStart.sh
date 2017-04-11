#!/bin/bash

cd build
make clean
make -j
sudo systemctl restart CORE2017-Vision.service
sudo systemctl status CORE2017-Vision.service