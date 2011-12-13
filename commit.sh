#!/bin/sh
HG_FOLDER="../hg-3piLib"
GIT_REPO="git://github.com/Tasssadar/3piLib.git"
HG_REPO="https://technika.junior.cz/hg/3pilib"

if [ $# -eq 0 ]; then
    echo "Ya have to enter revision number!"
else
    rm 3piLibPack.h
    echo "#define PI_LIB_VERSION $1\n" >> 3piLibPack.h
    ./packer.sh 0
    git commit -a -m [$1] -e
    git push master master
    cd $HG_FOLDER
    hg pull $GIT_REPO
    hg update
    hg push $HG_REPO
fi