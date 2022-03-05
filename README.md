# SmartRubbishBin（智能倾倒垃圾桶）
 
  
  
### 仓库结构（暂定）

- `ros`文件夹存放与ROS相关的代码
- `CarModel`文件夹存放小车SolidWorks模型
- `Tutorial`中记录一些学习笔记
- `Makefile`配置自动构建或执行自动化任务
- `.gitignore`中填写不需要被git管理的文件/文件夹


### 开发环境依赖

#### 软件
- ROS1-Noetic 完整版（带Gazebo等）
- SolidWorks 2021 + SW2URDF
- Python 3.8.10（低版本未测试）
- openCV 4.2.0（低版本未测试）
  
#### 硬件
- USB摄像头（Ubuntu能直驱）


### 食用指南

为了节省空间，仓库里只有源代码。

> 如果要生成ROS的可执行文件，
> 请进入`ros/`文件夹，执行`catkin_make`。
>
> 如果要生成……

将来可能有很多不同种类的文件/代码，
一个个像这样编译生成很麻烦
（当然现在只有ROS代码一种）。

因此在根目录下配置了一个`Makefile`，在其中填入了各种文件的编译方法，
只要在根目录下输入`make`命令，所有项目就可以一次性编译完成。

> 顺便，`Makefile`中也配置了清理功能，
> 只要在根目录下输入`make only_source`指令，
> 就可以删除编译结果，只保留源代码。


### TODO:

- [ ] 机器人 SolidWorks 建模
- [ ] 环境建模
- [ ] ROS SLAM
- [ ] GAZEBO 模拟
- [ ] OpenCV 识别垃圾桶
