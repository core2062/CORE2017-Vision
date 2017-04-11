#!/bin/bash

#DISPLAY=:0.0 XAUTHORITY=/home/pi/.Xauthority /usr/bin/feh -q -p -Z -F -R  60 -Y -D 5.0 -B white /home/pi/CORE2017-Vision/pitDisplay/slides
#DISPLAY=:0.0 XAUTHORITY=/home/pi/.Xauthority 
#sudo /home/pi/CORE2017-Vision/fsExpandCheck_jessie.sh
if xhost >& /dev/null;
then /usr/bin/libreoffice --nologo --invisible --minimized --nofirststartwizard --nodefault --norestore --show /home/pi/CORE2017-Vision/pitDisplay/slides.odp
fi