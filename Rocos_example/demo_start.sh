#!/bin/bash

cd build
cmake ..
if ! make -j8; then
    echo "编译失败！请检查错误信息。"
else
    cd bin
    # echo a | sudo -S ./demo
    sudo ./demo
   
fi



# fi
