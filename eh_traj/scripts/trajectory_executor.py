#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVROS Trajectory Executor Node v2.0 (No Custom Messages)

- 一个简化的、可复用的单机控制器。
- 控制接口:
    - Service (std_srvs/Trigger): 用于 takeoff, land, hover 等简单指令。
    - Topic (geometry_msgs/PoseStamped): 用于接收 goto 指令的目标点。
- 使用平滑的正弦轨迹执行点对点移动。
- 通过launch文件完全参数化，包括MAVROS命名空间。
- 无需任何自定义消息或服务，易于集成。
"""

import rospy
import math
from enum import Enum
import tf.transformations

# ROS 标准消息和服务
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

class FlightState(Enum):
    IDLE = 0          # 地面，未解锁
    ARMING = 1        # 正在解锁并切换OFFBOARD
    TAKING_OFF = 2    # 正在垂直起飞
    HOVERING = 3      # 空中悬停 (默认安全状态)
    MOVING = 4        # 正在执行GOTO轨迹
    LANDING = 5       # 正在执行自动降落

class TrajectoryExecutorNode:
    def __init__(self):
        rospy.init_node('trajectory_executor', anonymous=True)

        # --- 加载参数 ---
        self.namespace = rospy.get_param('~namespace', 'iris_0')
        self.takeoff_height = rospy.get_param('~takeoff_height', 1.5)
        self.max_vel = rospy.get_param('~max_velocity', 1.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        self.control_rate = rospy.get_param('~control_rate', 20.0)

        # --- 内部状态变量 ---
        self.flight_state = FlightState.IDLE
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.hover_pose = PoseStamped()
        
        # 轨迹状态变量
        self.trajectory_initialized = False
        self.trajectory_start_time = None
        self.trajectory_start_pose = None
        self.trajectory_duration = 0.0

        # --- ROS 接口 ---
        if self.namespace:
            self.namespace = '/' + self.namespace.strip('/')
        
        rospy.loginfo(f"[{self.namespace}] 初始化轨迹执行器...")

        # MAVROS 接口
        state_topic = f"{self.namespace}/mavros/state"
        pose_topic = f"{self.namespace}/mavros/local_position/pose"
        setpoint_topic = f"{self.namespace}/mavros/setpoint_position/local"
        arming_srv = f"{self.namespace}/mavros/cmd/arming"
        set_mode_srv = f"{self.namespace}/mavros/set_mode"

        rospy.wait_for_service(arming_srv)
        rospy.wait_for_service(set_mode_srv)
        
        self.arming_client = rospy.ServiceProxy(arming_srv, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(set_mode_srv, SetMode)
        self.state_sub = rospy.Subscriber(state_topic, State, self._state_cb)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_cb)
        self.pos_setpoint_pub = rospy.Publisher(setpoint_topic, PoseStamped, queue_size=1)
        
        # --- 控制接口 ---
        # 使用Topic接收GoTo指令
        self.goto_sub = rospy.Subscriber('~goto', PoseStamped, self._handle_goto_topic)
        # 使用Service处理简单指令
        self.takeoff_service = rospy.Service('~takeoff', Trigger, self._handle_takeoff)
        self.land_service = rospy.Service('~land', Trigger, self._handle_land)
        self.hover_service = rospy.Service('~hover', Trigger, self._handle_hover)
        # 发布当前状态，便于上层节点监控
        self.status_pub = rospy.Publisher('~status', String, queue_size=1)

        # 主控制循环
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)
        rospy.loginfo(f"[{self.namespace}] 轨迹执行器已就绪。")

    # --- 控制接口的回调函数 ---
    def _handle_goto_topic(self, msg):
        """Topic回调：接收GoTo指令"""
        if self.flight_state in [FlightState.HOVERING, FlightState.MOVING]:
            rospy.loginfo("收到GoTo指令 (via Topic)，开始移动...")
            self.target_pose = msg
            self.trajectory_initialized = False
            self._set_and_publish_state(FlightState.MOVING)
        else:
            rospy.logwarn("无法执行GoTo指令，飞机当前不处于悬停或移动状态。")

    def _handle_takeoff(self, req):
        if self.flight_state == FlightState.IDLE:
            rospy.loginfo("收到Takeoff指令 (via Service)，开始执行...")
            self._set_and_publish_state(FlightState.ARMING)
            return TriggerResponse(success=True, message="Takeoff sequence initiated.")
        return TriggerResponse(success=False, message="Cannot takeoff, not in IDLE state.")

    def _handle_land(self, req):
        if self.flight_state in [FlightState.HOVERING, FlightState.MOVING]:
            rospy.loginfo("收到Land指令 (via Service)，开始降落...")
            self._set_and_publish_state(FlightState.LANDING)
            return TriggerResponse(success=True, message="Landing sequence initiated.")
        return TriggerResponse(success=False, message="Cannot land, not in a flying state.")

    def _handle_hover(self, req):
        if self.flight_state in [FlightState.MOVING]:
            rospy.loginfo("收到Hover指令 (via Service)，中断移动并悬停。")
            self._enter_hover_state()
            return TriggerResponse(success=True, message="Hover command accepted.")
        return TriggerResponse(success=False, message="Cannot hover, not in MOVING state.")

    # --- MAVROS Callbacks ---
    def _state_cb(self, msg): self.current_state = msg
    def _pose_cb(self, msg): self.current_pose = msg

    # --- Main Control Loop & State Logic ---
    def _control_loop(self, event):
        if self.flight_state == FlightState.ARMING:
            self._arming_logic()
        elif self.flight_state == FlightState.TAKING_OFF:
            self._takeoff_logic()
        elif self.flight_state == FlightState.HOVERING:
            self._hover_logic()
        elif self.flight_state == FlightState.MOVING:
            self._moving_logic()
        elif self.flight_state == FlightState.LANDING:
            self._landing_logic()

    def _arming_logic(self):
        self.pos_setpoint_pub.publish(self.current_pose)
        if self.current_state.mode != "OFFBOARD":
            self.set_mode_client(custom_mode="OFFBOARD")
        elif not self.current_state.armed:
            self.arming_client(value=True)
        else:
            rospy.loginfo("已解锁并进入 OFFBOARD 模式，准备起飞。")
            self._set_and_publish_state(FlightState.TAKING_OFF)

    def _takeoff_logic(self):
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.pose.position.x = self.current_pose.pose.position.x
        target.pose.position.y = self.current_pose.pose.position.y
        target.pose.position.z = self.takeoff_height
        target.pose.orientation = self.current_pose.pose.orientation
        self.pos_setpoint_pub.publish(target)
        if abs(self.current_pose.pose.position.z - self.takeoff_height) < self.goal_tolerance:
            rospy.loginfo("起飞完成，进入悬停状态。")
            self._enter_hover_state()

    def _hover_logic(self):
        self.hover_pose.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(self.hover_pose)

    def _moving_logic(self):
        if not self.trajectory_initialized:
            self.trajectory_start_time = rospy.Time.now()
            self.trajectory_start_pose = self.current_pose.pose
            dist = self._distance(self.trajectory_start_pose.position, self.target_pose.pose.position)
            self.trajectory_duration = (dist / self.max_vel) * (math.pi / 2.0) if self.max_vel > 0.01 else 0.0
            self.trajectory_initialized = True
            rospy.loginfo(f"开始新轨迹: 距离={dist:.2f}m, 预计时间={self.trajectory_duration:.2f}s")
        
        elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
        dist_ratio = 1.0 if elapsed >= self.trajectory_duration else math.sin((elapsed / self.trajectory_duration) * math.pi / 2.0)
        
        start_pos, goal_pos = self.trajectory_start_pose.position, self.target_pose.pose.position
        next_pos = Point()
        next_pos.x = start_pos.x + dist_ratio * (goal_pos.x - start_pos.x)
        next_pos.y = start_pos.y + dist_ratio * (goal_pos.y - start_pos.y)
        next_pos.z = start_pos.z + dist_ratio * (goal_pos.z - start_pos.z)
        
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.pose.position = next_pos
        setpoint.pose.orientation = self.target_pose.pose.orientation
        self.pos_setpoint_pub.publish(setpoint)
        
        if self._distance(self.current_pose.pose.position, goal_pos) < self.goal_tolerance:
            rospy.loginfo("GoTo 目标点到达，进入悬停状态。")
            self._enter_hover_state()

    def _landing_logic(self):
        if self.current_state.mode != "AUTO.LAND":
            res = self.set_mode_client(custom_mode="AUTO.LAND")
        if not self.current_state.armed:
            rospy.loginfo("飞机已降落并解锁。返回IDLE状态。")
            self._set_and_publish_state(FlightState.IDLE)

    # --- Helper Functions ---
    def _set_and_publish_state(self, new_state):
        """设置新状态并发布它"""
        self.flight_state = new_state
        self.status_pub.publish(String(data=new_state.name))
        rospy.loginfo(f"状态切换 -> {new_state.name}")

    def _enter_hover_state(self):
        """进入悬停状态的辅助函数"""
        self.hover_pose = self.current_pose
        self._set_and_publish_state(FlightState.HOVERING)
        self.trajectory_initialized = False

    def _distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


if __name__ == '__main__':
    try:
        executor = TrajectoryExecutorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭。")
