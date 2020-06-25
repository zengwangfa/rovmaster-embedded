#!/bin/sh 

# 指定摄像头推流:
# 0不开启摄像头 
# 1开启摄像头1 
# 2开启摄像头2 
# 3两者都开启
CAMERA_ON=1

############# 摄像头视频推流参数 #############
CAMERA1_DEV="/dev/video0"                  #
CAMERA1_FPS=30                             #
CAMERA1_PORT=8085                          #
CAMERA1_RESO=1280x720                      #
                                           #
CAMERA2_DEV="/dev/video2"                  #
CAMERA2_FPS=30                             #
CAMERA2_PORT=8086                          #
CAMERA2_RESO=1280x720                      #
############################################

# 打印 IP地址
#IP_ADDR=$(hostname -I)
#echo "\033[36m[sh] My IP:$IP_ADDR\033[0m" 

# 打印系统时间
#ctime=$(date "+%Y-%m-%d %H:%M:%S")
#echo "\033[36m[sh] $ctime\033[0m"

case $CAMERA_ON in
    1)echo "\033[32m[sh] mjpg-streamer for camera1 \033[0m"  
	# 启动mjpg-streamer 推流 Camera1
	/usr/local/bin/mjpg_streamer \
	-i "/usr/local/lib/mjpg-streamer/input_uvc.so -d $CAMERA1_DEV -n -f $CAMERA1_FPS -r $CAMERA1_RESO" \
	-o "/usr/local/lib/mjpg-streamer/output_http.so -p $CAMERA1_PORT -w /usr/local/share/mjpg-streamer/www"
	;;

    2)echo "\033[32m[sh] mjpg-streamer for camera2 \033[0m"  
	# 启动mjpg-streamer 推流 Camera2
	/usr/local/bin/mjpg_streamer \
	-i "/usr/local/lib/mjpg-streamer/input_uvc.so -d $CAMERA2_DEV -n -f $CAMERA2_FPS -r $CAMERA2_RESO" \
	-o "/usr/local/lib/mjpg-streamer/output_http.so -p $CAMERA2_PORT -w /usr/local/share/mjpg-streamer/www"
	;;

    3)echo "\033[32m[sh] mjpg-streamer for camera1 \033[0m"   
	# 启动mjpg-streamer 推流 Camera1
	/usr/local/bin/mjpg_streamer \
	-i "/usr/local/lib/mjpg-streamer/input_uvc.so -d $CAMERA1_DEV -n -f $CAMERA1_FPS -r $CAMERA1_RESO" \
	-o "/usr/local/lib/mjpg-streamer/output_http.so -p $CAMERA1_PORT -w /usr/local/share/mjpg-streamer/www"
	
	echo "\033[32m[sh] mjpg-streamer for camera2 \033[0m" 
	# 启动mjpg-streamer 推流 Camera2
	/usr/local/bin/mjpg_streamer \
	-i "/usr/local/lib/mjpg-streamer/input_uvc.so -d $CAMERA2_DEV -n -f $CAMERA2_FPS -r $CAMERA2_RESO" \
	-o "/usr/local/lib/mjpg-streamer/output_http.so -p $CAMERA2_PORT -w /usr/local/share/mjpg-streamer/www"
	;;

	*)echo "\033[31m[sh] turn off video stream \033[0m"
	;;
esac

exit 0
