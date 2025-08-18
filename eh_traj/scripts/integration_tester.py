#!/usr.bin/env python3
# -*- coding: utf-8 -*-
# by wxc
"""
Integration Tester for Trajectory Executor (v1.3 - State-Based)

- A standalone ROS node to run a sequence of tests on a running trajectory_executor.
- Relies on the executor's status topic for robust state tracking.
- Verifies that the waypoint_reached topic is published correctly.
"""
import rospy
import threading
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped

class IntegrationTester:
    def __init__(self, namespace):
        rospy.init_node('integration_tester', anonymous=True)

        self.namespace = namespace
        
        # 修正服务和话题的前缀
        self.prefix = f"/trajectory_executor_{self.namespace}"

        # --- 内部状态与事件 ---
        self.current_drone_status = "UNKNOWN"
        self.status_lock = threading.Lock()
        
        # 使用 threading.Event 实现更可靠的等待/通知
        self.takeoff_completed_event = threading.Event()
        self.waypoint_reached_event = threading.Event()
        self.goto_completed_event = threading.Event()
        self.waypoint_signal_received = False # 用于验证辅助话题

        # --- ROS 接口 ---
        self.status_sub = rospy.Subscriber(f"{self.prefix}/status", String, self.status_callback)
        self.waypoint_reached_sub = rospy.Subscriber(f"{self.prefix}/waypoint_reached", Bool, self.waypoint_reached_callback)
        
        rospy.loginfo("等待连接到执行器服务...")
        try:
            rospy.wait_for_service(f"{self.prefix}/takeoff", timeout=10)
            rospy.wait_for_service(f"{self.prefix}/land", timeout=10)
            self.takeoff_client = rospy.ServiceProxy(f"{self.prefix}/takeoff", Trigger)
            self.land_client = rospy.ServiceProxy(f"{self.prefix}/land", Trigger)
            self.goto_pub = rospy.Publisher(f"{self.prefix}/goto", PoseStamped, queue_size=1)
            rospy.loginfo("服务和话题已连接！")
        except rospy.ROSException as e:
            rospy.logerr(f"连接执行器失败: {e}。请确保节点 '{self.prefix}' 正在运行。")
            rospy.signal_shutdown("Executor not found")
            return
            
        rospy.loginfo(f"将测试命名空间 '{self.namespace}' 下的执行器: {self.prefix}")

    def status_callback(self, msg):
        with self.status_lock:
            self.current_drone_status = msg.data
        
        # 根据状态变化设置事件
        if self.current_drone_status == "HOVERING":
            self.takeoff_completed_event.set()
        
        # GOTO任务完成的标志是状态变回 空中IDLE
        if self.current_drone_status == "IDLE" and self.takeoff_completed_event.is_set():
             self.goto_completed_event.set()


    def waypoint_reached_callback(self, msg):
        if msg.data:
            rospy.loginfo("检测到航点到达信号 (`waypoint_reached`=True)！")
            self.waypoint_signal_received = True
            self.waypoint_reached_event.set()

    def run_test_sequence(self):
        rospy.loginfo(">>> 步骤1: 发送起飞指令...")
        try:
            response = self.takeoff_client.call(TriggerRequest())
            if not response.success:
                rospy.logerr(f"起飞服务调用失败: {response.message}，测试终止。")
                return
            rospy.loginfo("起飞指令已发送。")
        except rospy.ServiceException as e:
            rospy.logerr(f"起飞服务调用异常: {e}")
            return

        rospy.loginfo(">>> 步骤2: 等待无人机起飞并进入 HOVERING 状态...")
        completed = self.takeoff_completed_event.wait(timeout=20.0) # 等待最多20秒
        if not completed:
            rospy.logerr("等待起飞超时！测试终止。")
            return
        rospy.loginfo("状态确认：无人机已进入 HOVERING。")
        
        # --- 发送GOTO指令前，重置事件和标志 ---
        self.waypoint_reached_event.clear()
        self.goto_completed_event.clear()
        self.waypoint_signal_received = False

        rospy.sleep(1.0) # 短暂稳定

        rospy.loginfo(">>> 步骤3: 发送目标点位 [2.0, 2.0, 1.5]...")
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = 2.0
        target_pose.pose.position.y = 2.0
        target_pose.pose.position.z = 1.5
        target_pose.pose.orientation.w = 1.0
        self.goto_pub.publish(target_pose)
        
        rospy.loginfo(">>> 步骤4: 等待无人机到达目标点 (状态变为 IDLE)...")
        completed = self.goto_completed_event.wait(timeout=25.0) # 等待最多25秒
        if not completed:
            rospy.logerr("等待 GOTO 完成超时！测试终止。")
            return
        rospy.loginfo("状态确认：无人机已完成GOTO并进入空中 IDLE。")

        # 验证 waypoint_reached 话题是否也被发布了
        if self.waypoint_signal_received:
            rospy.loginfo("验证成功：`waypoint_reached` 话题已正确发布。")
        else:
            rospy.logwarn("验证警告：`waypoint_reached` 话题未收到信号。")
        
        rospy.sleep(2.0) # 到达后悬停2秒

        rospy.loginfo(">>> 步骤5: 发送降落指令...")
        try:
            response = self.land_client.call(TriggerRequest())
            if not response.success:
                rospy.logerr("降落服务调用失败。")
            else:
                rospy.loginfo("降落指令已发送。")
        except rospy.ServiceException as e:
            rospy.logerr(f"降落服务调用异常: {e}")

        rospy.loginfo("\n" + "="*30 + "\n  所有测试步骤已执行完毕！\n" + "="*30)

if __name__ == '__main__':
    try:
        # 从参数服务器获取命名空间，如果没有则为空字符串
        uav_namespace = rospy.get_param("~namespace", "") 
        tester = IntegrationTester(namespace=uav_namespace)
        
        if rospy.is_shutdown():
             exit()

        rospy.sleep(1) # 等待所有连接建立
        tester.run_test_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断。")