#!/bin/sh 
#这个脚本的作用是:当修改了lib中文件，进行编译(包括WiringNP、easylogger、mjpg-streamer)


#编译标志位
# 0 不编译lib
# 1 编译WiringNP
# 2 编译easylogger
# 3 编译mjpg-streamer
COMPILE_FLAG="n"

echo "\033[36m1:WiringNP，2:easylogger，3:mjpg-streamer\033[0m"
echo "输入需要进行编译的lib(1 or 2 or 3):"
read COMPILE_FLAG

if [ "$COMPILE_FLAG"x = "1"x ]; then
    echo "=====================编译WiringNP=========================="
    cd ./lib/WiringNP/
    chmod 755 build
    ./build

elif [ "$COMPILE_FLAG"x = "2"x ]; then
    echo "=======================编译easylogger======================"
    cd ./lib/easylogger
    make
    sudo cp libeasylogger.so ..
    sudo cp libeasylogger.so /usr/lib/

elif [ "$COMPILE_FLAG"x = "3"x ]; then
    echo "=================编译mjpg-streamer及安装相关依赖================="
    cd ./lib/mjpg-streamer-experimental/
    make
    sudo make install
fi

exit 0
