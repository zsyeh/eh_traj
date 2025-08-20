#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class ServoTriggerNode:
    def __init__(self):
        # 初始化 ROS 节点，并从参数服务器获取参数
        rospy.init_node('servo_trigger_node', anonymous=True)
        # 获取无人机的命名空间，如果没有，则默认为空字符串
        self.namespace = rospy.get_param('~namespace', '')
        self.target_waypoint_count = rospy.get_param('~target_count', 8)
        self.current_waypoint_count = 0
        # 从参数服务器获取单个舵机角度，默认值为90.0
        self.servo_angle = rospy.get_param('~servo_angle', 90.0)
        # 根据命名空间构建话题名称
        if self.namespace:
            # 当命名空间不为空时，话题名为 /trajectory_executor_<namespace>/waypoint_reached
            waypoint_topic = f"/trajectory_executor_{self.namespace}/waypoint_reached"
        else:
            # 当命名空间为空时，话题名为 /trajectory_executor_/waypoint_reached
            waypoint_topic = "/trajectory_executor_/waypoint_reached"
            
        rospy.loginfo(f"Servo Trigger Node started in namespace '{self.namespace}'.")
        rospy.loginfo(f"Subscribing to topic: {waypoint_topic}")
        rospy.loginfo(f"Will release servo on waypoint {self.target_waypoint_count}.")

        # 创建舵机控制发布器
        self.servo_pub = rospy.Publisher("/arduino_ros", Point, queue_size=1)

        # 订阅无人机航点到达话题
        rospy.Subscriber(waypoint_topic, Bool, self._waypoint_callback)

    def _waypoint_callback(self, msg):
        if msg.data:
            self.current_waypoint_count += 1
            rospy.loginfo(f"Waypoint reached. Current count: {self.current_waypoint_count}")
            
            # 当航点计数达到目标值时，发布舵机指令
            if self.current_waypoint_count == self.target_waypoint_count:
                rospy.loginfo("Target waypoint count reached! Releasing servo.")
                servo_cmd = Point()
                servo_cmd.x = self.servo_angle
                servo_cmd.y = self.servo_angle
                servo_cmd.z = self.servo_angle
                self.servo_pub.publish(servo_cmd)
                
                # 为了避免重复发布，可以设置一个标志位或者将计数器重置
                # self.target_waypoint_count = -1 
                
def main():
    try:
        node = ServoTriggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()