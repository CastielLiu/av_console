#!/bin/sh

basepath=$(cd `dirname $0`; pwd)

rtkpath=${basepath}/../rtcm3.2

#echo ${rtkpath}

cd ${rtkpath}

./main


