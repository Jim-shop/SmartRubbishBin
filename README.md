# SmartRubbishBin（智能倾倒垃圾桶）


## 仓库结构（暂定）

- `📂Arduino`文件夹存放Arduino代码
- `📂CarModel`文件夹存放小车SolidWorks模型
- `📂ROS`文件夹存放与ROS相关的代码
- `📂Tutorial`中记录一些学习笔记
- `📄.gitignore`中填写不需要被git管理的文件/文件夹
- `📄Makefile`配置自动构建或执行自动化任务
- `📄README.md`就是本文件


## 开发环境

- Ubuntu == 20.04 ~ 20.04.4
- Arduino IDE >= 1.8.19
- ROS1-Noetic-Desktop-Full（自带OpenCV)
- ros-noetic-serial包
- SolidWorks == 2021
- Arduino Uno 一片
- L298N 一片
- 激光雷达一个

## 食用指南

### 1. 首先将仓库克隆到本地：

```bash
cd 父路径（自选）
git clone git@github.com:Jim-shop/SmartRubbishBin.git
```
> 没有配置公钥的话可以用HTTPS克隆。

这样，`git`会在自选的`父路径`下新建一个以这个仓库名命名的文件夹，然后将仓库内容以及文件历史信息等克隆到这个文件夹里。

注意，此仓库会用到一些预训练的神经网络，这些神经网络的权重文件比较大，不方便上传Github，请在网络上下载后手动放到相应地方。

### 2. 配置环境

#### （1）Arduino IDE 从官网下载安装
apt源得到的版本较为老旧。
#### （2）Ros Noetic和Serial包参考以下命令安装
```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-serial
```
#### （3）给设备起别名，防止设备号乱了
首先执行
```bash
lsusb
```
得到设备号如`10c4:ea60`。
进入`cd /etc/udev/rules.d`进入目录，新建一个`usb.rules`文件，内容为
```shell
KERNEL=="ttyUSB*",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",MODE:="0777",SYMLINK+="laser"
KERNEL=="ttyUSB*",ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="7523",MODE:="0777",SYMLINK+="uno"
```
然后重启udev：
```bash
service udev reload
service udev restart
```
重新插拔USB设备即可。



### 3. 编译
   
为了节省空间和保护隐私，仓库里只有源代码，要生成可执行程序，请参照以下操作：

#### （1）逐一构建
- 如果要生成ROS的可执行文件，请：
    ```bash
    cd 刚克隆下来的仓库路径/ros/
    catkin_make
    ```
- 如果要生成Arduino固件，请直接使用Arduino IDE打开ino文件进行编译烧录。

#### （2）批量构建
将来可能有很多不同种类的文件/代码（当然现在只有ROS代码和Arduino代码两种），逐一编译生成很麻烦。因此在仓库根目录下配置了一个`Makefile`，在其中填入了各种文件的编译方法，只要在根目录下输入：
```bash
cd 刚克隆下来的仓库路径/
make
```
这样所有项目就可以一次性编译完成。

> 顺便，`Makefile`中也配置了清理功能，
> 只要在根目录下执行：
> ```bash
> make only_source
> ```
> 就可以删除编译结果，只保留源代码。

### 4. 开发

参见[Git协作开发指南](Tutorial/Git协作开发指南.md)。
