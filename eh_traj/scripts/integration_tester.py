#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Integration Tester for Trajectory Executor (v1.2 - Logic Fix)

- A standalone ROS node to run a sequence of tests on a running trajectory_executor.
- Correctly resets the waypoint_reached flag before waiting.
"""
import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped

class IntegrationTester:
    def __init__(self, namespace):
        rospy.init_node('integration_tester', anonymous=True)

        self.namespace = namespace
        
        # 修正服务和话题的前缀以匹配 launch 文件
        if self.namespace:
            self.prefix = f"/trajectory_executor_{self.namespace}"
        else:
            self.prefix = "/trajectory_executor_"

        # --- 内部状态标志 ---
        self.current_drone_status = None
        self.takeoff_complete = False
        self.waypoint_reached = False

        # --- ROS 接口 ---
        self.status_sub = rospy.Subscriber(f"{self.prefix}/status", String, self.status_callback)
        self.waypoint_reached_sub = rospy.Subscriber(f"{self.prefix}/waypoint_reached", Bool, self.waypoint_reached_callback)
        self.takeoff_client = rospy.ServiceProxy(f"{self.prefix}/takeoff", Trigger)
        self.land_client = rospy.ServiceProxy(f"{self.prefix}/land", Trigger)
        self.goto_pub = rospy.Publisher(f"{self.prefix}/goto", PoseStamped, queue_size=1)

        rospy.loginfo("集成测试节点已初始化。")
        rospy.loginfo(f"将测试节点: {self.prefix}")

    def status_callback(self, msg):
        """监听无人机状态，用于判断起飞是否完成"""
        self.current_drone_status = msg.data
        if self.current_drone_status == "HOVERING" and not self.takeoff_complete:
            self.takeoff_complete = True
            rospy.loginfo("检测到无人机进入HOVERING状态，起飞完成！")

    def waypoint_reached_callback(self, msg):
        """监听航点到达话题"""
        if msg.data:
            # 只有当我们在等待时，才将标志位设为True
            if not self.waypoint_reached:
                self.waypoint_reached = True
                rospy.loginfo("检测到航点到达信号！")

    def run_test_sequence(self):
        """执行完整的测试序列"""
        rospy.loginfo("等待控制器服务可用...")
        try:
            rospy.wait_for_service(f"{self.prefix}/takeoff", timeout=10)
            rospy.wait_for_service(f"{self.prefix}/land", timeout=10)
            rospy.loginfo("服务已连接！")
        except rospy.ROSException:
            rospy.logerr(f"连接控制器服务超时。请确保服务 '{self.prefix}/takeoff' 存在。")
            return

        rospy.loginfo(">>> 步骤1: 发送起飞指令...")
        try:
            # ... (起飞部分代码不变) ...
            response = self.takeoff_client.call(TriggerRequest())
            if not response.success:
                rospy.logerr("起飞服务调用失败，测试终止。")
                return
            rospy.loginfo("起飞指令已发送。")
        except rospy.ServiceException as e:
            rospy.logerr(f"起飞服务调用异常: {e}")
            return

        rospy.loginfo(">>> 步骤2: 等待无人机起飞并悬停...")
        rate = rospy.Rate(2)
        while not self.takeoff_complete and not rospy.is_shutdown():
            rospy.loginfo(f"    当前状态: {self.current_drone_status}, 等待 HOVERING...")
            rate.sleep()
        
        if rospy.is_shutdown(): return
        
        # --- 核心修复：在发送GOTO指令前，重置航点到达标志 ---
        rospy.loginfo("重置航点到达标志位。")
        self.waypoint_reached = False

        rospy.loginfo(">>> 步骤3: 发送目标点位 [2.0, 2.0, 1.5]...")
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = 2.0
        target_pose.pose.position.y = 2.0
        target_pose.pose.position.z = 1.5
        target_pose.pose.orientation.w = 1.0
        self.goto_pub.publish(target_pose)
        
        rospy.loginfo(">>> 步骤4: 等待无人机到达目标点...")
        while not self.waypoint_reached and not rospy.is_shutdown():
            rospy.loginfo("    等待 waypoint_reached 话题...")
            rate.sleep()
        
        if rospy.is_shutdown(): return

        rospy.loginfo(">>> 步骤5: 发送降落指令...")
        try:
            # ... (降落部分代码不变) ...
            response = self.land_client.call(TriggerRequest())
            if not response.success:
                rospy.logerr("降落服务调用失败。")
            else:
                rospy.loginfo("降落指令已发送。")
        except rospy.ServiceException as e:
            rospy.logerr(f"降落服务调用异常: {e}")

        rospy.loginfo("="*20 + "\n所有测试步骤已执行完毕！\n" + "="*20)

if __name__ == '__main__':
    try:
        uav_namespace = rospy.get_param("~namespace", "iris_0")
        tester = IntegrationTester(namespace=uav_namespace)
        rospy.sleep(1)
        tester.run_test_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断。")