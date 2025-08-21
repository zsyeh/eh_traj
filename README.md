### 1.测试舵机
先拆掉桨叶，给无人机上电看舵机拉杆是否能够闭合，若不能闭合则在上电自动复位后 将金属抽杆重新安装 确保在复位状态下为闭合状态

### 2.通过NoMachine远程桌面校准无人机
通过Nmap扫无人机的ip，通过QGC进行校准
```
nmap -p 22 192.168.x.0/24 | grep -B 4 open
```

### 3.启动程序
通过ssh连接两架无人机
`ssh phoenixtech@192.168.x.x`
密码ptuav


修改雷达ip，将lidar_configs下的ip改为192.168.1.1xx，xx为雷达后面sn码最后两位
```
cd ~/livox_ws/src/livox_ros_driver2/config
vim MID360_config.json
```

启动sh脚本
```
./position_tmux.sh
```

主机
```
roslaunch eh_traj master_tcp_planner.launch

# 仿真只需要修改ns参数 roslaunch eh_traj master_tcp_planner.launch ns:=iris_0
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

从机
```
roslaunch eh_traj slave_tcp_planner.launch master_host:=<修改为主机的ip地址>
# 仿真只需要修改ns参数 roslaunch eh_traj master_tcp_planner.launch ns:=iris_1 
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
（若ttyUSB0没有则改为ttyUSB1）

等待指令完成

打印位置的消息，确认`/mavros/`话题正常
```
rostopic echo /mavros/vision_pose/pose
```
状态检查 
向前x增加 
向左y增加
向上z增加
并检查里程计是否收敛


`tmux a` 连接会话 通过ctrl + B + 数字切换不同窗口检查节点是否完整

检查无误后
确保 主机必须放在场地的右侧起飞点 从机放在左侧起飞点 

在主机执行起飞命令
`rosservice call /dual_uav_tcp_planner/start_full_mission "{}"`
注意：该命令执行后无人机会立即起飞 无需遥控器干预 务必保证遥控器处于开机状态 情况异常立刻切定点或自稳模式接管必要时emergency kill


## 补充
比赛代码基本原理概述

### livox_ws 为四个节点 分别为 
- 雷达驱动
- fastlio
- mavros
- vision_pose pub


### traj_ws 主要有以下节点
- 底层控制器
- 上层规划器 （读取点位yaml文件）
- tcp主从通信（和上层规划期公用一个py脚本和一个节点）
- 规划监视器和舵机控制器（用于跟踪任务执行进度 在到达点位时释放吊运货物）
- 串口通信节点

二次开发请修改点位waypoints.yaml的yaml文件 并计算出释放货物的索引并修改master_tcp_planner.launch和slave_tcp_planner.launch文件中的参数target_count







仿真
```
roslaunch eh_traj master_tcp_planner.launch ns:=iris_0
 
roslaunch eh_traj slave_tcp_planner.launch ns:=iris_1  

rosservice call /dual_uav_tcp_planner/start_full_mission "{}"

rosrun rosserial_python serial_node.py /dev/ttyUSB0


```


<img width="1137" height="923" alt="image" src="https://github.com/user-attachments/assets/04880d83-0bf0-47ad-af1b-5b4893f43b2a" />















<img width="1204" height="1106" alt="image" src="https://github.com/user-attachments/assets/f204f6a5-8e98-42e9-b315-c72db67c16b1" />
启动
`roslaunch eh_traj executor.launch `

指定ns参数
`roslaunch eh_traj executor.launch namespace:=iris_1`


测试

```
rosservice list | grep trajectory_executor
rosservice call /trajectory_executor/takeoff "{}"
rosservice call /trajectory_executor/land "{}"

rostopic pub -1 /trajectory_executor/goto geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

rostopic pub -1 /trajectory_executor/goto geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

```
一键执行任务命令 one key takeoff
`rosservice call /waypoint_planner/start_full_mission "{}"`
<img width="1239" height="1255" alt="image" src="https://github.com/user-attachments/assets/563650f8-b0a5-47f2-8acc-f304dc3edf67" />
