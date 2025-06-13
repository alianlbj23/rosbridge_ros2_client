#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

class RobotArmRelay(Node):
    """
    Subscribe to /move_group/status and /joint_states.
    Only after receiving a SUCCEEDED status (GoalStatus.SUCCEEDED == 3)
    will joint_states messages be republished on the 'robot_arm' topic.
    """
    def __init__(self):
        super().__init__('robot_arm_relay')
        self.reached = False

        # Publisher for the filtered joint_states
        self.arm_pub = self.create_publisher(JointState, 'robot_arm', 10)

        # Subscribe to move_group/status to detect when the goal is reached
        self.create_subscription(
            GoalStatusArray,
            '/move_group/status',
            self.status_callback,
            10
        )

        # Subscribe to joint_states; only forward when reached=True
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('robot_arm_relay node started, waiting for goal to succeed...')

    def status_callback(self, msg: GoalStatusArray):
        # Check if any goal in the status_list has succeeded
        for status in msg.status_list:
            if status.status == GoalStatus.SUCCEEDED:
                if not self.reached:
                    self.get_logger().info('Received SUCCEEDED status, now forwarding joint_states → /robot_arm')
                self.reached = True
                return

    def joint_callback(self, msg: JointState):
        # Only forward joint_states once the goal is reached
        if self.reached:
            self.arm_pub.publish(msg)
            # Optionally reset reached if you only want the first message:
            # self.reached = False

def main():
    rclpy.init()
    node = RobotArmRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
