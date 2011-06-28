#!/bin/sh
if [ $# -eq 0 ]; then
    echo "Ya have to enter revision number!"
else
    rm 3piLibPack.h
    echo -e "#define PI_LIB_VERSION $1\n" >> 3piLibPack.h
    ./packer.sh 0
    git commit -a -m [$1] -e
    git push master master
fi