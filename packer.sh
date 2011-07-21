#!/bin/bash
if [ $# -ne 1 ] ; then #  [ $1 -ne 0 ] ; then
    rm 3piLibPack.h
fi

files=( common.h motors.h buzzer.h time.h sensors.h display.h rs232.h i2c.h buttons.h init.h )
for file in ${files[@]}
do
    cat $file >> 3piLibPack.h
    echo  >> 3piLibPack.h
done
