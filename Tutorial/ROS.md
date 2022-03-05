# ROS 功能包开发流程
  

## 一、准备工作空间
   
1. 将命令行工作路径移动到所需用作工作空间的文件夹
   
    ```bash
    cd 自定义工作空间文件夹名
    ```

2. 在那个工作空间中新建一个`src`文件夹，用来保存源代码

    ```bash
    mkdir src
    ```

3. 调用ROS工具初始化工作空间 *（注意此时工作路径不是在`src`中）*
   
    ```bash
    catkin_make
    ```

   此时会在src文件夹下生成一个`CMakeLists.txt`的符号链接


## 二、创建功能包

进入`src`目录，创建ROS功能包，并添加依赖

```bash
cd src
catkin_create_pkg 自定义功能包名 依赖项1 依赖项2 ...
```

开发Cpp应用，需要添加`roscpp`依赖项，Python则需要`rospy`依赖项。一般来说还要引入`std_msgs`标准消息库。

> 执行完`catkin_create_pkg`后，工作空间中的文件就会多起来，其中：
> 
> - `build`文件夹存放已经编译完成的ROS功能包
> - `devel`文件夹中存放运行功能包所需的一些环境配置文件
> - `src`文件夹就是我们自行创建的那个，
>   但是里面会新建一个功能包文件夹，
>   文件夹的名字就是我们刚刚自定义的功能包名。


## 三、编写功能包

> 功能包文件夹结构：
>
> - `include`文件夹中保存Cpp的头文件
> - `src`文件夹中保存Cpp的源代码
> 
> 以上是ROS构造工具的默认结构。
> 为了方便，不要改动文件夹名和它们的用途。
> 
> 另外，还可以自行新建文件夹：
> - `scripts`，用来存放Python脚本。
> - `launch`，用来存放launch文件，用于批量启动节点。

### （一）Cpp程序编写

1. 在功能包文件夹下`src`中新建Cpp源文件（名字自定义），填入代码。简明示例如下：
   
    ```cpp
    #include "ros/ros.h"
    // 头文件位于/opt/ros/noetic/include/**

    int main(int argc, char *argv[])
    {
        /* 第一步：执行ROS节点初始化
        * ros::init([传递命令行参数],节点名称)
        */
        ros::init(argc, argv, "example");

        /* 第二步（可选）：创建ROS节点句柄
        */
        ros::NodeHandle n;

        ROS_INFO("hello world!");

        /* 离别时：报平安
        */
        return 0;
    }
    ```

2. 编辑功能包文件夹下的`CMakeLists.txt`，参照其注释，修改编译配置：

    ```cmake
    add_executable(编译成什么名字 src/源文件名.cpp)
    target_link_libraries(编译成什么名字
        ${catkin_LIBRARIES}
    )
    ```

3. 进入工作空间根目录，执行编译

    ```bash
    cd 工作空间文件夹
    catkin_make
    ```

### （二）Python脚本编写

1. 可以在功能包文件夹中新建一个`scripts`目录用以存放脚本。

    ```bash
    cd 功能包文件夹
    mkdir scripts
    ```

2. 在`scripts`文件夹中新建Python脚本，名字自拟。示例如下：

    ```python
    #! /bin/python3
    """
    上面一行按格式填写Python解释器路径，
    这样，当赋予这个脚本文件可执行权限时，
    在控制台敲入此文件名并回车，
    控制台就能根据此行找到对应的解释器，
    自动调用它执行这个脚本，而不需要我们手工指定。
    """

    import rospy #rospy在安装ros时已经安装好

    if __name__ == "__main__":
        # 第一步：给定节点名，初始化节点
        rospy.init_node("examplePY")
        rospy.loginfo("Hello world.")
    ```

3. 为Python文件添加可执行权限
   
    ```bash
    chmod +x 脚本文件名.py
    ```

4. 编辑功能包文件夹下的`CMakeLists.txt`，参照其注释，修改编译配置：

    ```cmake
    catkin_install_python(PROGRAMS
        scripts/脚本文件名.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```

    > Python虽然不需要编译，但是可以通过此步生成环境配置，
    > 方便互相调用。


## 四、执行功能包

### （一）手工执行

1. 首先系统中要有一个启动的`roscore`。如果没有，执行：
   
    ```bash
    roscore
    ```

    > 不使用“小技巧”，直接执行`roscore`，会占用一个控制台窗口。
    >
    > 若还要使用控制台应用，需要新开一个控制台窗口。

2. 进入工作空间文件夹，载入`devel`文件夹下的环境配置脚本

    ```bash
    cd 工作空间
    source ./devel/setup.bash
    ```

    > 如果使用的是`Zsh`，就载入`setup.zsh`。
    >
    > 注意，对于每个控制台，在运行下一步之前都要载入这个脚本。
    > 如果懒得每次输入，可以把这指令加入到`~/.bashrc`，`~/.zshrc`等里面。 

3. 执行
   
    ```bash
    rosrun 包名 可执行文件
    ```

    > 对于Cpp应用，`可执行文件`是`编译成的名称`；
    > 
    > 对于Python脚本，`可执行文件`就是`脚本名称.py`

### （二）批处理执行

0. **【准备工作】** 在功能包文件夹下新建`launch`文件夹，在其中添加launch文件（名称自拟），文件内容示例：
      
    ```xml
    <launch>
        <!--pkg=功能包 type=被运行的功能包文件 name=为节点命名 output=设置日志的输出目标-->
        <node pkg="Example" type="eg.py" name="launch_eg_py" output="screen" />
        <node pkg="Example" type="Example_node" name="launch_eg_cpp" output="screen" />

        <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    </launch>
    ```

1. 不需要特意启动`roscore`。`roslaunch`会自动启动它。

2. 进入工作空间文件夹，载入`devel`文件夹下的环境配置脚本

    ```bash
    cd 工作空间
    source ./devel/setup.bash
    ```

    > 如果使用的是`Zsh`，就载入`setup.zsh`。
    >
    > 注意，对于每个控制台，在运行下一步之前都要载入这个脚本。
    > 如果懒得每次输入，可以把这指令加入到`~/.bashrc`，`~/.zshrc`等里面。 

3. 运行launch文件

    ```bash
    roslaunch 包名 launch文件名
    ```
