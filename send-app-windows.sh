#!/bin/sh
# Author: Serg Kolo
# Description: Send two different applications to different windows
set -x

#LEFT=$(zenity --entry --text="What to put on the left?")
#RIGHT=$(zenity --entry --text="What to put on the right?")

LEFT='left'
RIGHT='right'
#$LEFT &
#$RIGHT &
#sleep 1

if [ $? -eq 0 ]; then
    # Get title of the left window
    LEFT_TITLE="$(wmctrl -l | grep -i $LEFT | awk '{$1=$2=$3="";print}')"
    # Get title of the right window
    RIGHT_TITLE="$(wmctrl -l | grep -i $RIGHT | awk '{$1=$2=$3="";print }')"
#printf "THIS IS A TEST %s",$LEFT_TITLE
#printf "THIS IS A TEST 2 %s",$RIGHT_TITLE
    wmctrl -r $LEFT_TITLE -e 0,1680,0,960,1080

    sleep 0.5

    wmctrl -r $RIGHT_TITLE -e 0,2600,0,960,1080

fi


#wmctrl -r $LEFT_TITLE -e 0,0,0,960,1080
#wmctrl -r $RIGHT_TITLE -e 0,1080,0,960,1080
