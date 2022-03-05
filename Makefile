MakefilePath	:= $(abspath $(lastword $(MAKEFILE_LIST)))
RootPath		:= $(MakefilePath:Makefile=)

.PHONY:
	default only_source ros_build

default: ros_build Makefile

ros_build: Makefile
	cd $(RootPath)/ros && catkin_make 

only_source: Makefile
	-cd $(RootPath)/ros && rm -r devel build
