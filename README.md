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
