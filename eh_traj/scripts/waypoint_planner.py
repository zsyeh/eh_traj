#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fully Automated Synchronous Dual-Drone Waypoint Planner (v2.0)

- 从参数服务器加载两条无人机的轨迹。
- 提供单一服务来启动全自动任务。
- 自动调用 takeoff 服务，并等待两机起飞完成。
- 确认起飞完成后，开始同步执行YAML文件中的航点任务。
- 所有阶段均通过话题进行同步。
"""
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

class WaypointPlanner:
    def __init__(self):
        rospy.init_node('waypoint_planner')

        # --- 加载参数 ---
        self.traj_a = rospy.get_param('~trajectory_a')
        self.traj_b = rospy.get_param('~trajectory_b')
        self.frame_id = rospy.get_param('~target_frame_id', 'map')

        if len(self.traj_a) != len(self.traj_b):
            rospy.logerr("错误：两条轨迹的航点数量必须相同！程序将退出。")
            return

        # --- 内部状态变量 ---
        self.state = "IDLE"  # IDLE, TAKING_OFF, MISSION_ACTIVE, FINISHED
        self.current_waypoint_index = 0
        
        # 跟踪每个无人机的状态
        self.drone_a_airborne = False
        self.drone_b_airborne = False
        self.drone_a_reached_current_wp = False
        self.drone_b_reached_current_wp = False
        
        # 定义两架无人机的命名空间
        self.ns_a = "iris_0"
        self.ns_b = "iris_1"
        self.prefix_a = f"/trajectory_executor_{self.ns_a}"
        self.prefix_b = f"/trajectory_executor_{self.ns_b}"

        # --- ROS 接口 ---
        # 指令发布者
        self.goto_pub_a = rospy.Publisher(f'{self.prefix_a}/goto', PoseStamped, queue_size=1)
        self.goto_pub_b = rospy.Publisher(f'{self.prefix_b}/goto', PoseStamped, queue_size=1)

        # 状态订阅者
        self.status_sub_a = rospy.Subscriber(f'{self.prefix_a}/status', String, self.status_cb_a)
        self.waypoint_reached_sub_a = rospy.Subscriber(f'{self.prefix_a}/waypoint_reached', Bool, self.waypoint_reached_cb_a)
        
        self.status_sub_b = rospy.Subscriber(f'{self.prefix_b}/status', String, self.status_cb_b)
        self.waypoint_reached_sub_b = rospy.Subscriber(f'{self.prefix_b}/waypoint_reached', Bool, self.waypoint_reached_cb_b)
        
        # 服务客户端 (用于起飞)
        self.takeoff_client_a = rospy.ServiceProxy(f'{self.prefix_a}/takeoff', Trigger)
        self.takeoff_client_b = rospy.ServiceProxy(f'{self.prefix_b}/takeoff', Trigger)

        # 任务控制服务
        self.start_service = rospy.Service('~start_full_mission', Trigger, self.handle_start_mission)

        rospy.loginfo("全自动双机指挥官已就绪。")
        rospy.loginfo("等待底层控制器服务...")
        try:
            self.takeoff_client_a.wait_for_service(timeout=5.0)
            self.takeoff_client_b.wait_for_service(timeout=5.0)
            rospy.loginfo("底层控制器服务已连接。")
            rospy.loginfo("调用 '~start_full_mission' 服务以开始全自动任务。")
        except rospy.ROSException:
            rospy.logerr("连接底层控制器服务超时！请确保executor节点正在运行。")

    # --- 状态回调处理 ---
    def status_cb_a(self, msg):
        # 仅在TAKING_OFF阶段关心状态变化
        if self.state == "TAKING_OFF" and msg.data == 'HOVERING' and not self.drone_a_airborne:
            self.drone_a_airborne = True
            rospy.loginfo(f"同步确认：无人机 {self.ns_a} 已起飞并悬停。")
            self.check_and_start_waypoints()

    def status_cb_b(self, msg):
        if self.state == "TAKING_OFF" and msg.data == 'HOVERING' and not self.drone_b_airborne:
            self.drone_b_airborne = True
            rospy.loginfo(f"同步确认：无人机 {self.ns_b} 已起飞并悬停。")
            self.check_and_start_waypoints()

    def waypoint_reached_cb_a(self, msg):
        if msg.data and self.state == "MISSION_ACTIVE" and not self.drone_a_reached_current_wp:
            rospy.loginfo(f"无人机 {self.ns_a} 已到达航点 {self.current_waypoint_index}。")
            self.drone_a_reached_current_wp = True
            self.check_and_advance_waypoint()

    def waypoint_reached_cb_b(self, msg):
        if msg.data and self.state == "MISSION_ACTIVE" and not self.drone_b_reached_current_wp:
            rospy.loginfo(f"无人机 {self.ns_b} 已到达航点 {self.current_waypoint_index}。")
            self.drone_b_reached_current_wp = True
            self.check_and_advance_waypoint()

    # --- 核心任务逻辑 ---
    def handle_start_mission(self, req):
        if self.state != "IDLE":
            return TriggerResponse(success=False, message=f"任务已在状态 {self.state}，无法开始。")
        
        rospy.loginfo("收到全自动任务指令！开始进入起飞阶段...")
        self.state = "TAKING_OFF"
        try:
            self.takeoff_client_a.call()
            self.takeoff_client_b.call()
            rospy.loginfo("已向双机发送起飞指令，等待悬停确认...")
            return TriggerResponse(success=True, message="起飞指令已发送")
        except rospy.ServiceException as e:
            self.state = "IDLE" # 失败则回滚状态
            rospy.logerr(f"调用起飞服务失败: {e}")
            return TriggerResponse(success=False, message="调用起飞服务失败")

    def check_and_start_waypoints(self):
        # 当两架无人机都已升空
        if self.drone_a_airborne and self.drone_b_airborne:
            rospy.loginfo("="*20)
            rospy.loginfo("双机均已升空！开始执行航点任务...")
            rospy.loginfo("="*20)
            self.state = "MISSION_ACTIVE"
            self.current_waypoint_index = 0
            self.publish_current_waypoint()

    def check_and_advance_waypoint(self):
        # 当两架无人机都到达当前航点
        if self.drone_a_reached_current_wp and self.drone_b_reached_current_wp:
            rospy.loginfo(f"同步确认：航点 {self.current_waypoint_index} 已被双机共同到达！")
            
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.traj_a):
                rospy.loginfo(f"发布下一个航点: {self.current_waypoint_index}...")
                self.publish_current_waypoint()
            else:
                rospy.loginfo("="*20)
                rospy.loginfo("所有航点均已完成！任务圆满结束。")
                rospy.loginfo("="*20)
                self.state = "FINISHED"

    def publish_current_waypoint(self):
        self.drone_a_reached_current_wp = False
        self.drone_b_reached_current_wp = False

        point_a = self.traj_a[self.current_waypoint_index]
        point_b = self.traj_b[self.current_waypoint_index]

        self.goto_pub_a.publish(self.create_pose_stamped(point_a))
        self.goto_pub_b.publish(self.create_pose_stamped(point_b))
        rospy.loginfo(f"指令已发送: {self.ns_a} -> {point_a}, {self.ns_b} -> {point_b}")

    def create_pose_stamped(self, point):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
        pose.pose.orientation.w = 1.0
        return pose

if __name__ == '__main__':
    try:
        WaypointPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass