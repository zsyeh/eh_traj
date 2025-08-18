#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# by wxc
"""
MAVROS Trajectory Executor Node v3.1 (Robust & Simplified)

改动要点：
- 移除了 stop_at_waypoint 参数，逻辑更清晰：到达目标点后总是进入悬停并报告完成。
- “连续飞行”的功能由上层规划器通过快速发送新航点实现。
- 状态机优化：
  - 地面未解锁: IDLE
  - 起飞后/指令悬停: HOVERING
  - GOTO 移动完成后: 发布 waypoint_reached=True，然后状态切换为 IDLE (空中)，
    这为上层规划器提供了清晰、统一的“任务段完成”信号，并兼容之前的 tcp_slave_bridge。
"""

import rospy
import math
from enum import Enum

# ROS 消息与服务
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String, Bool


class FlightState(Enum):
    IDLE = "IDLE"          # 地面未解锁 / 空中完成任务，等待指令
    ARMING = "ARMING"        # 正在解锁并切换 OFFBOARD
    TAKING_OFF = "TAKING_OFF"  # 正在垂直起飞
    HOVERING = "HOVERING"      # 空中悬停（由起飞或手动指令进入）
    MOVING = "MOVING"        # 执行 GOTO 轨迹
    LANDING = "LANDING"      # 自动降落


class TrajectoryExecutorNode:
    def __init__(self):
        rospy.init_node('trajectory_executor', anonymous=True)

        # --- 参数 ---
        self.namespace = rospy.get_param('~namespace', '')
        self.takeoff_height = rospy.get_param('~takeoff_height', 1.5)
        self.max_vel = rospy.get_param('~max_velocity', 1.8)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        self.control_rate = rospy.get_param('~control_rate', 50.0)

        # --- 内部状态 ---
        self.flight_state = FlightState.IDLE
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.hover_pose = PoseStamped()

        # 轨迹执行状态
        self.trajectory_initialized = False
        self.trajectory_start_time = None
        self.trajectory_start_pose = None
        self.trajectory_duration = 0.0

        # GOTO 航点到达信号控制
        self.waypoint_task_active = False

        # --- 话题/服务命名 ---
        ns_prefix = f"/{self.namespace}" if self.namespace else ""
        log_prefix = f"[{self.namespace}]" if self.namespace else "[Executor]"

        rospy.loginfo(f"{log_prefix} 初始化轨迹执行器 v3.1...")

        # MAVROS 接口
        state_topic = f"{ns_prefix}/mavros/state"
        pose_topic = f"{ns_prefix}/mavros/local_position/pose"
        setpoint_topic = f"{ns_prefix}/mavros/setpoint_position/local"
        arming_srv = f"{ns_prefix}/mavros/cmd/arming"
        set_mode_srv = f"{ns_prefix}/mavros/set_mode"

        rospy.loginfo(f"{log_prefix} 等待 MAVROS 服务...")
        rospy.wait_for_service(arming_srv)
        rospy.wait_for_service(set_mode_srv)
        rospy.loginfo(f"{log_prefix} MAVROS 服务已连接。")

        self.arming_client = rospy.ServiceProxy(arming_srv, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(set_mode_srv, SetMode)
        self.state_sub = rospy.Subscriber(state_topic, State, self._state_cb)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_cb)
        self.pos_setpoint_pub = rospy.Publisher(setpoint_topic, PoseStamped, queue_size=1)

        # 控制接口
        self.goto_sub = rospy.Subscriber('~goto', PoseStamped, self._handle_goto_topic)
        self.takeoff_service = rospy.Service('~takeoff', Trigger, self._handle_takeoff)
        self.land_service = rospy.Service('~land', Trigger, self._handle_land)
        self.hover_service = rospy.Service('~hover', Trigger, self._handle_hover)

        # 状态/反馈
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.waypoint_reached_pub = rospy.Publisher('~waypoint_reached', Bool, queue_size=1)

        # 首次发布初始状态
        self.status_pub.publish(String(data=self.flight_state.value))

        # 主循环
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)
        rospy.loginfo(f"{log_prefix} 轨迹执行器已就绪。")

    def _state_cb(self, msg): self.current_state = msg
    def _pose_cb(self, msg): self.current_pose = msg

    def _handle_goto_topic(self, msg):
        if self.flight_state in [FlightState.HOVERING, FlightState.IDLE] and self.current_state.armed:
            rospy.loginfo(f"收到GoTo指令，目标: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}]")
            self.target_pose = msg
            self.trajectory_initialized = False
            self.waypoint_task_active = True # 标记GOTO任务开始
            self._set_state(FlightState.MOVING)
        else:
            rospy.logwarn(f"无法执行GoTo，当前状态: {self.flight_state.value} (armed: {self.current_state.armed})")

    def _handle_takeoff(self, req):
        if self.flight_state == FlightState.IDLE and not self.current_state.armed:
            rospy.loginfo("收到Takeoff指令，开始执行...")
            self._set_state(FlightState.ARMING)
            return TriggerResponse(success=True, message="Takeoff sequence initiated.")
        return TriggerResponse(success=False, message=f"Cannot takeoff, state is {self.flight_state.value}")

    def _handle_land(self, req):
        if self.flight_state in [FlightState.HOVERING, FlightState.MOVING, FlightState.IDLE] and self.current_state.armed:
            rospy.loginfo("收到Land指令，开始降落...")
            self._set_state(FlightState.LANDING)
            return TriggerResponse(success=True, message="Landing sequence initiated.")
        return TriggerResponse(success=False, message=f"Cannot land, state is {self.flight_state.value}")

    def _handle_hover(self, req):
        if self.flight_state == FlightState.MOVING:
            rospy.loginfo("收到Hover指令，中断移动并悬停。")
            self._enter_hover_state()
            return TriggerResponse(success=True, message="Hover command accepted.")
        return TriggerResponse(success=False, message=f"Cannot hover, state is {self.flight_state.value}")

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
        elif self.flight_state == FlightState.IDLE and self.current_state.armed:
            # 如果是空中IDLE状态，保持悬停
            self.pos_setpoint_pub.publish(self.current_pose)

    def _arming_logic(self):
        self.pos_setpoint_pub.publish(self.current_pose)
        if self.current_state.mode != "OFFBOARD":
            self.set_mode_client(custom_mode="OFFBOARD")
        elif not self.current_state.armed:
            self.arming_client(value=True)
        else:
            rospy.loginfo("已解锁并进入 OFFBOARD 模式，准备起飞。")
            self._set_state(FlightState.TAKING_OFF)

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
            effective_vel = max(self.max_vel, 0.01)
            self.trajectory_duration = dist / effective_vel
            self.trajectory_initialized = True
            rospy.loginfo(f"开始新轨迹: 距离={dist:.2f} m, 预计耗时={self.trajectory_duration:.2f} s")

        elapsed = (rospy.Time.now() - self.trajectory_start_time).to_sec()
        s = max(0.0, min(1.0, elapsed / self.trajectory_duration if self.trajectory_duration > 1e-6 else 1.0))

        start_pos, goal_pos = self.trajectory_start_pose.position, self.target_pose.pose.position
        next_pos = Point(
            x=start_pos.x + s * (goal_pos.x - start_pos.x),
            y=start_pos.y + s * (goal_pos.y - start_pos.y),
            z=start_pos.z + s * (goal_pos.z - start_pos.z)
        )
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.pose.position = next_pos
        setpoint.pose.orientation = self.target_pose.pose.orientation
        self.pos_setpoint_pub.publish(setpoint)

        if self._distance(self.current_pose.pose.position, goal_pos) < self.goal_tolerance:
            rospy.loginfo("到达目标点。")
            if self.waypoint_task_active:
                self.waypoint_reached_pub.publish(Bool(data=True))
                rospy.loginfo("发布航点到达信号: True")
                self.waypoint_task_active = False # 确保只为GOTO任务发布一次
            
            # 任务完成，进入空中IDLE状态等待新指令
            self._enter_air_idle_state()

    def _landing_logic(self):
        if self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(custom_mode="AUTO.LAND")
        if not self.current_state.armed:
            rospy.loginfo("飞机已降落并上锁。返回地面IDLE状态。")
            self._set_state(FlightState.IDLE)

    def _set_state(self, new_state):
        if self.flight_state != new_state:
            self.flight_state = new_state
            self.status_pub.publish(String(data=new_state.value))
            rospy.loginfo(f"状态切换 -> {new_state.value}")

    def _enter_hover_state(self):
        self.hover_pose = self.current_pose
        self._set_state(FlightState.HOVERING)

    def _enter_air_idle_state(self):
        """ GOTO任务完成后，进入空中IDLE状态，功能上等同于悬停 """
        self._set_state(FlightState.IDLE)

    @staticmethod
    def _distance(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

if __name__ == '__main__':
    try:
        executor = TrajectoryExecutorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭。")