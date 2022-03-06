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

#### 1. 首先将仓库克隆到本地：

```bash
cd 父路径（自选）
git clone https://gitee.com/Jim_shop/SmartRubbishBin.git
```

这样，`git`会在`父路径`下新建一个以这个仓库名命名的文件夹，然后将仓库内容以及文件历史信息等克隆到这个文件夹里。

#### 2. 编译
   
为了节省空间和保护隐私，仓库里只有源代码，要生成可执行程序，请参照以下操作：

##### （1）逐一构建
- 如果要生成ROS的可执行文件，请：
    ```bash
    cd 刚克隆下来的仓库路径/ros/
    catkin_make
    ```

##### （2）批量构建
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

#### 3. 开发

使用Git进行协作开发的流程一般是
1. 首先从远程服务器拉取最新的主分支
2. 从主分支创建本地新分支，切换到这个新分支上
3. 按常规的方式进行开发
4. 新分支开发有了进展，在新分支内本地提交
5. 开发测试正常后，签出到主分支，整合，推送

##### （1）拉取
```bash
cd 刚克隆下来的仓库路径/
git pull
```

##### （2）创建并签出到新分支
```bash
cd 刚克隆下来的仓库路径/
git checkout -b 新分支名（自拟）
```

##### （3）开发
切换分支后，先暂时不用管Git，直接按照常规的方法进行开发就行了，不认识Git的时候怎么开发，现在就怎么开发。

##### （4）提交
```bash
cd 刚克隆下来的仓库路径/
git commit -a 需要提交的文件/目录 -m "关于本次提交的说明（自拟）"
```
这样就把开发的成果保存到Git版本管理库中了。
> `需要提交的文件/目录`应该有哪些？
>
> 应该只有源代码。你编译出的内容一是可能存在兼容性问题，比如只能在你的电脑上执行，其他电脑上会出现问题，二是为了保持仓库简洁，出于这两点原因，请不要上传。

> **但是每次手敲文件好麻烦呀！**
> 
> 其实可以把不需要提交的文件/目录名添加到仓库根目录下的`.gitignore`文件中，这样这些文件就会被Git忽略。
> 于是就可以使用这条命令：
> ```bash
> git commit -a -m "关于本次提交的说明（自拟）"
> ```
> 把`需要提交的文件/目录`省略，这代表仓库根目录。
> 这样Git就会把`.gitignore`文件中记录的文件/目录之外的所有文件添加到Git版本管理库中。

##### （5）推送
开发测试正常后，可以签出到主分支，整合，推送。
```
cd 刚克隆下来的仓库路径/
git checkout master
git merge 之前自拟的新分支名
git push
```


#### 工程协作开发指南


### TODO:

- [ ] 机器人 SolidWorks 建模
- [ ] 环境建模
- [ ] ROS SLAM
- [ ] GAZEBO 模拟
- [ ] OpenCV 识别垃圾桶
