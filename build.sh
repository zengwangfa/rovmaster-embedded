#!/bin/sh 
#这个脚本的作用是自动安装相关的软件、安装相关的库
#需要root权限来运行这个脚本

echo "======================使编译脚本可执行======================"
chmod +x compile.sh

echo "======================使推流脚本可执行======================"
chmod +x ./tools/video.sh

echo "========================安装WiringNP========================"
cd ./lib/WiringNP/
chmod 755 build
./build

echo "=====================安装音视频相关依赖===================="
# 更新软件列表
sudo apt-get update

# 安装软件 cmake libjpeg8-dev
sudo apt-get install -y cmake libjpeg8-dev 
cd ../mjpg-streamer-experimental/
make
sudo make install

# 安装 alsa-lib音频库
sudo apt-get install libasound2-dev

echo "=======================编译easylogger======================"
cd ../easylogger
make
sudo cp libeasylogger.so ..
sudo cp libeasylogger.so /usr/lib/

echo "\033[35m=======================构建完成=======================\033[0m" 
exit 0
