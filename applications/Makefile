#-------------------------配置区域-----------------------
#DIR指的是.o文件和.c文件所在的目录
DIR=.
CROSS_COMPILE = arm-linux-gnueabihf-
#BIN指的是最终生成的目标对象名(包括路径)，它可以是可执行程序、动态链接库或静态链接库
BIN=$(DIR)/libapplications.a

#SHARE特指链接生成动态链接库对象时的编译选项
SHARE=--share

#CFLAG即compile flag，表示在编译时所加入的选项参数
#参数包括  
#-Wall  : 编译后显示所有警告信息
#-g     : 编译时加入调试信息，以便之后可以用gdb调试
#-fPIC  : 编译动态链接库时加入的选项
#-I./inc: -I选项指定从哪个目录寻找头文件，在这指定之后，在源文件中包含头文件就可以使用<>，这里./inc即为指定的目录
CFLAG=-Wall -g -lwiringPi -lpthread -lm -I../lib/easylogger/inc -I../lib/WiringNP/wiringPi

#LFLAG即library flag，表示链接生成可执行程序时所要链接的所有库的选项参数
#-L./lib : -L指示动态/静态链接库所在的目录，这里./lib即所在的目录
#-l      : -l指示动态/静态链接库的名字，注意: 这里的库名字并不包括前缀(lib)和后缀(.a或.so)
#$(SHARE) : 取SHARE变量对应的动态链接库链接选项--share
LFLAG=

#CC即编译链接命令gcc -o 用于编译或者链接动态库以及可执行程序
CC=arm-linux-gnueabi-gcc -o

#AR即ar -cr ，ar -cr是专门链接生成静态库的命令
#-c : 即create，创建静态库
#-r : 即replace，当静态库改变时，替代原有静态库
AR=arm-linux-gnueabi-ar -r

#最终用的是哪个链接命令
#链接生成动态库和可执行程序用CC
#链接生成静态库用AR
CO=$(AR)

#-------------------------以下为通用不变区域-----------------------
#SRC指的是指定目录下的所有.c文件名，OBJ指的是指定目录下的所有.o文件名
SRC=$(wildcard $(DIR)/*.c)
OBJ=$(patsubst %.c, %.o, $(SRC))

#链接命令
all:$(BIN)
$(BIN):$(OBJ)
	$(CO) $@ $^ $(LFLAG)

#编译命令
$(DIR)/%.o:$(DIR)/%.c
	$(CC) $@ -c $< $(CFLAG)

#清除无用文件
.PHONY:clean
clean:
	rm -f $(OBJ) $(BIN)
