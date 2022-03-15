# 获取这个Makefile的绝对路径
MakefilePath	:= $(abspath $(lastword $(MAKEFILE_LIST)))
# 获取Makefile所在的根目录（不带斜杠）
RootPath		:= $(MakefilePath:/Makefile=)

.PHONY:
	default only_source ros_build

default: ros_build Makefile

ros_build: Makefile
	cd $(RootPath)/ros && catkin_make 

only_source: Makefile
	-cd $(RootPath)/ros && rm -r devel build devel_isolated build_isolated
