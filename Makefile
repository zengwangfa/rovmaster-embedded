.PHONY:all clean

DIR=.
# 排除目录
exclude_dirs := .git .vscode easylogger lib user mjpg-streamer-master
# 显示深度为1的子目录
dirs := $(shell find . -maxdepth 1 -type d)
# 去掉获取到目录名称前面的./
dirs := $(basename $(patsubst ./%, %, $(dirs)))
# 过滤指定目录
dirs := $(filter-out $(exclude_dirs), $(dirs))
# 获取系统当前时间
ctime = $(shell date "+%Y-%m-%d %H:%M:%S")

all:
	# 删除目标文件，为了生成新的目标
	rm -f user/rovmaster

	# make -C:进入指定的目录下执行make命令，命令前加了@即不会打印命令本身
	@$(foreach N,$(dirs),make -C $(N);)

	# 将静态库文件拷贝到lib目录下
	@$(foreach N,$(dirs),cp ./$(N)/*.a ./lib;)

	@make -C ./user
	@cp ./user/rovmaster .
	
	# 生成反汇编
	# @objdump  -d rovmaster > rovmaster.dis

	# 打印提示信息
	@echo "\033[35m[$(ctime)] make success > rovmaster,run:\033[0m" 
	@echo "\033[35msudo ./rovmaster\033[0m" 
clean:
	#make clean -C: 进入指定目录下执行make clean命令
	@$(foreach N,$(dirs),make clean -C $(N);)
	@make clean -C ./user
	rm -f rovmaster
