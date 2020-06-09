#!/bin/sh

rtkpath=$(cd `rospack find av_console`/../rtcm3.2; pwd)
#echo ${rtkpath}

cd ${rtkpath}

./main


