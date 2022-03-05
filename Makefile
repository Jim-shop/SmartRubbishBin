MakefilePath	:= $(abspath $(lastword $(MAKEFILE_LIST)))
RootPath		:= $(MakefilePath:Makefile=)

.PHONY:
	default clean ros

default: ros Makefile

ros: Makefile
	cd $(RootPath)/ros && catkin_make 

clean: Makefile
	-rm *.tmp *.temp *.cache
