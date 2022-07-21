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

- Arduino IDE >= 1.8.19
- ROS1-Noetic 完整版
- SolidWorks >= 2021
- OpenCV >= 4.2.0

## 食用指南

### 1. 首先将仓库克隆到本地：

```bash
cd 父路径（自选）
git clone https://gitee.com/Jim_shop/SmartRubbishBin.git
```

这样，`git`会在`父路径`下新建一个以这个仓库名命名的文件夹，然后将仓库内容以及文件历史信息等克隆到这个文件夹里。

神经网络的权重文件比较大，不方便上传Github，请在网络上下载后手动放到相应地方。

### 2. 编译
   
为了节省空间和保护隐私，仓库里只有源代码，要生成可执行程序，请参照以下操作：

#### （1）逐一构建
- 如果要生成ROS的可执行文件，请：
    ```bash
    cd 刚克隆下来的仓库路径/ros/
    catkin_make
    ```

#### （2）批量构建
将来可能有很多不同种类的文件/代码（当然现在只有ROS代码一种），逐一编译生成很麻烦。因此在仓库根目录下配置了一个`Makefile`，在其中填入了各种文件的编译方法，只要在根目录下输入：
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

### 3. 开发

参见[Git协作开发指南](Tutorial/Git协作开发指南.md)。
