# ROS 功能包管理命令


## 1. 增

创建新的ROS功能包
```bash
catkin_create_pkg 自定义包名 依赖包
```

安装 ROS功能包
```bash
sudo apt install xxx
```


## 2. 删

删除某个功能包
```bash
sudo apt purge xxx
```


## 3. 查

列出所有功能包
```bash
rospack list
```

查找某个功能包是否存在，如果存在返回安装路径
```bash
rospack find 包名
```

进入某个功能包
```bash
roscd 包名
```

列出某个包下的文件
```bash
rosls 包名
```

搜索某个功能包
```bash
apt search xxx
```


## 4. 改

修改功能包文件（需要安装 Vim）
```bash
rosed 包名 文件名
```


## 5. 执行

### (1) roscore
   
roscore 是 ROS 的系统先决条件节点和程序的集合， 必须运行 roscore 才能使 ROS 节点进行通信。

```bash
roscore
```
或指定端口号
```bash
roscore -p xxxx
```

roscore 将启动:
- ros master
- ros 参数服务器
- rosout 日志节点

### (2) rosrun
 
 运行指定的ROS节点
 ```bash
 rosrun 包名 可执行文件名
 ```

### (3) roslaunch
 执行某个包下的 launch 文件
 ```bash
 roslaunch 包名 launch文件名
 ```
