# shj2
森罗于此，万象予终
本项目提供一个头车自主/遥控行驶条件下的多车编队系统。

# 实车实验情况下使用说明（依据作者的实车环境）
## 启动前准备
```cpp
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
```
## 普通模式
script中driver0用于0号车，driver1用于通常跟随车，driver4用于节点加入时的特定跟随车
```cpp
zsh vs_ws/script/driver0.sh(然后输入密码启动imu和雷达)
roslaunch scout_bringup base0.launch(启动底盘及相关底层算法)
roslaunch trailing_mpc trailing_mpc.launch
```
## 队形变换模式
```cpp
首先需要根据自己的需求在cubic_spline_planner中填写对应的队形容器
可以通过直接在trailing_mpc.launch中修改formaiton_type来初始化不同队形
也可以通过使用rostopic pub /formation_type 来通过命令修改队形
```
## 编队重构模式
和普通编队模式使用方法相同
需要注意以下几个额外变化
zsh vs_ws/script/driver**4**.sh(待加入车辆使用3号车，在其agx_ws用driver改了一个driver4)
roslaunch scout_bringup base**4**.launch(启动底盘及相关底层算法)

# 仿真环境下使用说明
1. self_gps2xy中已经将odom的z字段填充了该车ID
2. **注意：重构功能需要同时修改self_gps2xy、cubic_spline_planner以及em_like模块**
3. 目前在编队加入车辆时，加入车辆的初始ID为4，对应目前的ReconfigurationCallback

# 一般使用
1. 启动trailing_mpc包下的vr.launch、virtual_base.launch等文件，里面提供了多种搭配的虚拟底盘的启动，如果想测试通常编队，推荐使用vr4.launch，它启动了仿真环境rviz以及四辆车的虚拟底盘，包括一辆头车和三辆跟随车
2. 启动trailing_mpc包下的trailing_mpc0.launch，这是头车的自主
3. 启动trailing_mpc包下的em_like1.launch等，后面的数字代表各自的ID，头车ID为0
4. 在启动完后会显示四个rviz窗口，点击上方的pose estimate按钮可以在地图中为各个车辆设置其起始位置，每个窗口的按钮对应各自的位置
5. 在设置好之后，任意一个rviz里应该都会显示每个车的规划轨迹，这时到0号车的rviz中点击上方的紫色按钮为编队设置目标点，即可开始编队的自主行驶
6. （如果不想使用头车自主驾驶，可以参考ros自带的turtlesim包中的键盘控制那个功能，把/cmd_vel话题重映射到头车对应的控制话题，把编队的头车变成遥控模式，后面的跟随车会自动跟随）

## 编队重构模式
```cpp
//首先需要初始化启动所有车辆的底盘和算法（包括待加入编队的节点4）
使用控制模块下的vr4add.launch来启动0，1，2，4号底盘（4号底盘是特意设计用来进行节点加入的）
    
//为他们指定初始位置

//在终端输入节点加入命令
首先rostopic pub /reID来指定重构模式，以及修改所有成员的ID
然后关闭rostopic pub /reID(或者pub -1 就自动关了)
```
**在实车上需要额外修改底盘和传感器node的相关节点名称ID**

```cpp
//首先需要初始化启动所有车辆的底盘和算法

//在终端输入节点退出命令
rostopic pub /reID来指定重构模式，以及修改所有成员的ID,对应车会自动下线
关闭rostopic pub /reID(或者pub -1 就自动关了)
```




